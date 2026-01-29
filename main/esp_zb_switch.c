/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier:  LicenseRef-Included
 *
 * Zigbee HA_on_off_switch Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_endpoint.h"
#include "zcl/esp_zigbee_zcl_humidity_meas.h"
#include "nwk/esp_zigbee_nwk.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "esp_zb_switch.h"
#include <stdlib.h>

static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}
};

static const char *TAG = "ESP_ZB_ON_OFF_SWITCH";
static bool s_factory_reset_requested;
static uint8_t s_last_binding_total;
static int64_t s_last_binding_total_update_us;
static bool s_relay_state;

/* Fake sensor values (ZCL units): temperature in 0.01°C (int16), humidity in 0.01% (uint16) */
static int16_t s_fake_temp_centi_c = 2300;
static uint16_t s_fake_humi_centi_pct = 4500;
static uint16_t s_fake_humi_min_centi_pct = ESP_ZB_ZCL_REL_HUMIDITY_MEASUREMENT_MIN_MEASURED_VALUE_MINIMUM;
static uint16_t s_fake_humi_max_centi_pct = 10000; /* 100.00% */
static uint16_t s_fake_humi_tolerance_centi_pct = 50; /* 0.50% */
static bool s_fake_reporting_started;

/* Retry interval for BDB network steering after failure */
#define ESP_ZB_STEERING_RETRY_DELAY_MS (3000)
#define ESP_ZB_STEERING_RETRY_DELAY_S  (ESP_ZB_STEERING_RETRY_DELAY_MS / 1000)

static const char *zb_nwk_cmd_status_to_string(uint8_t status)
{
    switch ((esp_zb_nwk_command_status_t)status) {
    case ESP_ZB_NWK_COMMAND_STATUS_NO_ROUTE_AVAILABLE:
        return "NO_ROUTE_AVAILABLE";
    case ESP_ZB_NWK_COMMAND_STATUS_TREE_LINK_FAILURE:
        return "TREE_LINK_FAILURE";
    case ESP_ZB_NWK_COMMAND_STATUS_NONE_TREE_LINK_FAILURE:
        return "NON_TREE_LINK_FAILURE";
    case ESP_ZB_NWK_COMMAND_STATUS_LOW_BATTERY_LEVEL:
        return "LOW_BATTERY_LEVEL";
    case ESP_ZB_NWK_COMMAND_STATUS_NO_ROUTING_CAPACITY:
        return "NO_ROUTING_CAPACITY";
    case ESP_ZB_NWK_COMMAND_STATUS_NO_INDIRECT_CAPACITY:
        return "NO_INDIRECT_CAPACITY";
    case ESP_ZB_NWK_COMMAND_STATUS_INDIRECT_TRANSACTION_EXPIRY:
        return "INDIRECT_TRANSACTION_EXPIRY";
    case ESP_ZB_NWK_COMMAND_STATUS_TARGET_DEVICE_UNAVAILABLE:
        return "TARGET_DEVICE_UNAVAILABLE";
    case ESP_ZB_NWK_COMMAND_STATUS_TARGET_ADDRESS_UNALLOCATED:
        return "TARGET_ADDRESS_UNALLOCATED";
    case ESP_ZB_NWK_COMMAND_STATUS_PARENT_LINK_FAILURE:
        return "PARENT_LINK_FAILURE";
    case ESP_ZB_NWK_COMMAND_STATUS_VALIDATE_ROUTE:
        return "VALIDATE_ROUTE";
    case ESP_ZB_NWK_COMMAND_STATUS_SOURCE_ROUTE_FAILURE:
        return "SOURCE_ROUTE_FAILURE";
    case ESP_ZB_NWK_COMMAND_STATUS_MANY_TO_ONE_ROUTE_FAILURE:
        return "MANY_TO_ONE_ROUTE_FAILURE";
    case ESP_ZB_NWK_COMMAND_STATUS_ADDRESS_CONFLICT:
        return "ADDRESS_CONFLICT";
    case ESP_ZB_NWK_COMMAND_STATUS_VERIFY_ADDRESS:
        return "VERIFY_ADDRESS";
    case ESP_ZB_NWK_COMMAND_STATUS_PAN_IDENTIFIER_UPDATE:
        return "PAN_IDENTIFIER_UPDATE";
    case ESP_ZB_NWK_COMMAND_STATUS_NETWORK_ADDRESS_UPDATE:
        return "NETWORK_ADDRESS_UPDATE";
    case ESP_ZB_NWK_COMMAND_STATUS_BAD_FRAME_COUNTER:
        return "BAD_FRAME_COUNTER";
    case ESP_ZB_NWK_COMMAND_STATUS_BAD_KEY_SEQUENCE_NUMBER:
        return "BAD_KEY_SEQUENCE_NUMBER";
    case ESP_ZB_NWK_COMMAND_STATUS_UNKNOWN_COMMAND:
        return "UNKNOWN_COMMAND";
    default:
        return "UNKNOWN";
    }
}

typedef struct {
    uint16_t dst_addr;
    uint8_t start_index;
} zb_bind_dump_ctx_t;

static void zb_format_ieee_addr(char *out, size_t out_len, const esp_zb_ieee_addr_t addr_le)
{
    /* esp_zb_* APIs use little-endian representation; print as conventional big-endian. */
    (void)snprintf(out, out_len,
                   "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                   addr_le[7], addr_le[6], addr_le[5], addr_le[4], addr_le[3], addr_le[2], addr_le[1], addr_le[0]);
}

static void zb_binding_table_dump_cb(const esp_zb_zdo_binding_table_info_t *table_info, void *user_ctx)
{
    zb_bind_dump_ctx_t *ctx = (zb_bind_dump_ctx_t *)user_ctx;

    if (!table_info || !ctx) {
        return;
    }

    ESP_LOGI(TAG, "Binding table rsp: status=0x%02x total=%u index=%u count=%u (dst=0x%04hx)",
             table_info->status, table_info->total, table_info->index, table_info->count, ctx->dst_addr);

    s_last_binding_total = table_info->total;
    s_last_binding_total_update_us = esp_timer_get_time();

    const esp_zb_zdo_binding_table_record_t *rec = table_info->record;
    while (rec) {
        char src_ieee[24] = {0};
        zb_format_ieee_addr(src_ieee, sizeof(src_ieee), rec->src_address);

        if (rec->dst_addr_mode == 0x01) { /* 16-bit group address */
            ESP_LOGI(TAG, "  bind: src=%s ep=%u cluster=0x%04hx -> group=0x%04hx",
                     src_ieee, rec->src_endp, rec->cluster_id, rec->dst_address.addr_short);
        } else if (rec->dst_addr_mode == 0x03) { /* 64-bit + endpoint */
            char dst_ieee[24] = {0};
            zb_format_ieee_addr(dst_ieee, sizeof(dst_ieee), rec->dst_address.addr_long);
            ESP_LOGI(TAG, "  bind: src=%s ep=%u cluster=0x%04hx -> dst=%s ep=%u",
                     src_ieee, rec->src_endp, rec->cluster_id, dst_ieee, rec->dst_endp);
        } else {
            ESP_LOGI(TAG, "  bind: src=%s ep=%u cluster=0x%04hx -> dst_mode=0x%02x",
                     src_ieee, rec->src_endp, rec->cluster_id, rec->dst_addr_mode);
        }
        rec = rec->next;
    }

    if (table_info->status == 0x00 && table_info->count > 0 &&
        (uint16_t)table_info->index + (uint16_t)table_info->count < (uint16_t)table_info->total) {
        /* Fetch the next page. */
        zb_bind_dump_ctx_t *next = (zb_bind_dump_ctx_t *)calloc(1, sizeof(zb_bind_dump_ctx_t));
        if (next) {
            next->dst_addr = ctx->dst_addr;
            next->start_index = (uint8_t)(table_info->index + table_info->count);
            esp_zb_zdo_mgmt_bind_param_t req = {
                .dst_addr = next->dst_addr,
                .start_index = next->start_index,
            };
            esp_zb_zdo_binding_table_req(&req, zb_binding_table_dump_cb, next);
        }
    }

    free(ctx);
}

static void zb_request_binding_table_dump(uint8_t start_index)
{
    const uint16_t self_short = esp_zb_get_short_address();
    if (self_short == 0xFFFF) {
        ESP_LOGW(TAG, "Binding table dump skipped: not joined (short=0xFFFF)");
        return;
    }

    zb_bind_dump_ctx_t *ctx = (zb_bind_dump_ctx_t *)calloc(1, sizeof(zb_bind_dump_ctx_t));
    if (!ctx) {
        ESP_LOGW(TAG, "Binding table dump skipped: OOM");
        return;
    }
    ctx->dst_addr = self_short;
    ctx->start_index = start_index;

    esp_zb_zdo_mgmt_bind_param_t req = {
        .dst_addr = ctx->dst_addr,
        .start_index = ctx->start_index,
    };
    esp_zb_zdo_binding_table_req(&req, zb_binding_table_dump_cb, ctx);
}

static void relay_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_OUTPUT_IO_RELAY_TEST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(GPIO_OUTPUT_IO_RELAY_TEST, 0);
    s_relay_state = false;
}

static void relay_set(bool on)
{
    s_relay_state = on;
    gpio_set_level(GPIO_OUTPUT_IO_RELAY_TEST, on ? 1 : 0);
    ESP_LOGI(TAG, "Relay(test LED) -> %s", on ? "ON" : "OFF");
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG,
                        "Received message: error status(%d)", message->info.status);

    if (message->info.dst_endpoint == HA_ONOFF_RELAY_ENDPOINT &&
        message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
        message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
        message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
        bool on = message->attribute.data.value ? *(bool *)message->attribute.data.value : false;
        relay_set(on);
    }
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        return zb_attribute_handler((const esp_zb_zcl_set_attr_value_message_t *)message);
    default:
        ESP_LOGD(TAG, "Zigbee action callback 0x%x", callback_id);
        return ESP_OK;
    }
}

static void sensor_send_one_shot_reports(void)
{
    /* Send reports to coordinator (short 0x0000), endpoint 1 by default */
    const uint16_t coordinator_short = 0x0000;

    esp_zb_lock_acquire(portMAX_DELAY);

    esp_zb_zcl_report_attr_cmd_t rep = {0};
    rep.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
    rep.zcl_basic_cmd.dst_addr_u.addr_short = coordinator_short;
    rep.zcl_basic_cmd.dst_endpoint = 1;
    rep.zcl_basic_cmd.src_endpoint = HA_TEMP_HUMI_SENSOR_ENDPOINT;
    rep.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    rep.dis_default_resp = 1;
    rep.manuf_specific = 0;
    rep.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;

    rep.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    rep.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
    (void)esp_zb_zcl_report_attr_cmd_req(&rep);

    rep.clusterID = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT;
    rep.attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
    (void)esp_zb_zcl_report_attr_cmd_req(&rep);

    esp_zb_lock_release();
}

static void sensor_fake_update_and_report_cb(uint8_t param)
{
    (void)param;

    /* Simple fake waveform: temp 23.00 -> 28.00 -> 23.00 ... ; humidity 45% -> 55% -> 45% ... */
    static int dir = 1;
    s_fake_temp_centi_c += (int16_t)(dir * 10); /* 0.10°C step */
    s_fake_humi_centi_pct += (uint16_t)(dir * 20); /* 0.20% step */
    if (s_fake_temp_centi_c >= 2800 || s_fake_temp_centi_c <= 2300) {
        dir *= -1;
    }
    if (s_fake_humi_centi_pct > 5500) {
        s_fake_humi_centi_pct = 5500;
    } else if (s_fake_humi_centi_pct < 4500) {
        s_fake_humi_centi_pct = 4500;
    }

    esp_zb_lock_acquire(portMAX_DELAY);
    (void)esp_zb_zcl_set_attribute_val(HA_TEMP_HUMI_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                      &s_fake_temp_centi_c, false);
    (void)esp_zb_zcl_set_attribute_val(HA_TEMP_HUMI_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                                      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
                                      &s_fake_humi_centi_pct, false);
    esp_zb_lock_release();

    ESP_LOGI(TAG, "Fake sensor: temp=%.2fC hum=%.2f%%",
             (double)s_fake_temp_centi_c / 100.0, (double)s_fake_humi_centi_pct / 100.0);

    sensor_send_one_shot_reports();

    /* Repeat every 10 seconds */
    esp_zb_scheduler_alarm((esp_zb_callback_t)sensor_fake_update_and_report_cb, 0, 10 * 1000);
}

static void zb_buttons_handler(switch_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SWITCH_FACTORY_RESET_CONTROL) {
        if (!s_factory_reset_requested) {
            s_factory_reset_requested = true;
            ESP_LOGW(TAG, "Button long-press: factory reset requested (erase zb_storage and restart)");
            esp_zb_factory_reset();
        }
        return;
    }
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL) {
        /* implemented light switch toggle functionality */
        esp_zb_zcl_on_off_cmd_t cmd_req;
        cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID;

        const int64_t now_us = esp_timer_get_time();
        if ((now_us - s_last_binding_total_update_us) > 5 * 1000 * 1000) {
            esp_zb_ieee_addr_t ieee = {0};
            esp_zb_get_long_address(ieee);
            char ieee_str[24] = {0};
            zb_format_ieee_addr(ieee_str, sizeof(ieee_str), ieee);

            ESP_LOGI(TAG, "Toggle dst mode: DST_ADDR_ENDP_NOT_PRESENT (uses APS binding table). self=%s short=0x%04hx ep=%u cluster=0x%04hx",
                     ieee_str, esp_zb_get_short_address(), (unsigned)HA_ONOFF_SWITCH_ENDPOINT, (unsigned)ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
            zb_request_binding_table_dump(0);
        } else if (s_last_binding_total == 0) {
            ESP_LOGW(TAG, "No bindings in table (last check). Toggle will not reach anyone unless you configure binding/group addressing.");
        }

        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_on_off_cmd_req(&cmd_req);
        esp_zb_lock_release();
        ESP_EARLY_LOGI(TAG, "Send 'on_off toggle' command");
    }
}

static esp_err_t deferred_driver_init(void)
{
    ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), zb_buttons_handler), ESP_FAIL, TAG,
                        "Failed to initialize switch driver");
    relay_gpio_init();
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering (join)");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                /* If the stored network is gone (e.g. coordinator forgot us), re-steer/rejoin. */
                ESP_LOGI(TAG, "Start network steering (rejoin)");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
        } else {
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
            if (!s_factory_reset_requested) {
                s_factory_reset_requested = true;
                ESP_LOGW(TAG, "Request factory reset to clear Zigbee storage and retry");
                esp_zb_factory_reset();
            }
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Joined network successfully (PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            zb_request_binding_table_dump(0);
            if (!s_fake_reporting_started) {
                s_fake_reporting_started = true;
                esp_zb_scheduler_alarm((esp_zb_callback_t)sensor_fake_update_and_report_cb, 0, 3 * 1000);
            }
        } else {
            ESP_LOGW(TAG, "Network steering failed (status: %s), retry in %ds", esp_err_to_name(err_status),
                     ESP_ZB_STEERING_RETRY_DELAY_S);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, ESP_ZB_STEERING_RETRY_DELAY_MS);
        }
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE: {
        const esp_zb_zdo_device_unavailable_params_t *p =
            (const esp_zb_zdo_device_unavailable_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (p) {
            char ieee_str[24] = {0};
            zb_format_ieee_addr(ieee_str, sizeof(ieee_str), p->long_addr);
            ESP_LOGW(TAG, "ZDO device unavailable: short=0x%04hx ieee=%s", p->short_addr, ieee_str);
        } else {
            ESP_LOGW(TAG, "ZDO device unavailable: (no params)");
        }
        break;
    }
    case ESP_ZB_NLME_STATUS_INDICATION: {
        const esp_zb_zdo_signal_nwk_status_indication_params_t *p =
            (const esp_zb_zdo_signal_nwk_status_indication_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (p) {
            ESP_LOGW(TAG, "NLME status: nwk_status=0x%02x (%s) addr=0x%04hx unknown_cmd=0x%02x",
                     p->status, zb_nwk_cmd_status_to_string(p->status), p->network_addr, p->unknown_command_id);
        } else {
            ESP_LOGW(TAG, "NLME status: (no params)");
        }
        break;
    }
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    /* Endpoint 1: On/Off switch (client) */
    esp_zb_on_off_switch_cfg_t switch_cfg = ESP_ZB_DEFAULT_ON_OFF_SWITCH_CONFIG();
    esp_zb_cluster_list_t *switch_clusters = esp_zb_on_off_switch_clusters_create(&switch_cfg);
    esp_zb_endpoint_config_t switch_ep_cfg = {
        .endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0,
    };
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(ep_list, switch_clusters, switch_ep_cfg));

    /* Endpoint 2: On/Off light (server) as test relay */
    esp_zb_on_off_light_cfg_t relay_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_cluster_list_t *relay_clusters = esp_zb_on_off_light_clusters_create(&relay_cfg);
    esp_zb_endpoint_config_t relay_ep_cfg = {
        .endpoint = HA_ONOFF_RELAY_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
        .app_device_version = 0,
    };
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(ep_list, relay_clusters, relay_ep_cfg));

    /* Endpoint 3: Temperature sensor (server) + add humidity measurement cluster */
    esp_zb_temperature_sensor_cfg_t temp_cfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
    esp_zb_cluster_list_t *sensor_clusters = esp_zb_temperature_sensor_clusters_create(&temp_cfg);

    esp_zb_attribute_list_t *hum_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    ESP_ERROR_CHECK(esp_zb_humidity_meas_cluster_add_attr(hum_attr_list, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &s_fake_humi_centi_pct));
    ESP_ERROR_CHECK(esp_zb_humidity_meas_cluster_add_attr(hum_attr_list, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &s_fake_humi_min_centi_pct));
    ESP_ERROR_CHECK(esp_zb_humidity_meas_cluster_add_attr(hum_attr_list, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &s_fake_humi_max_centi_pct));
    ESP_ERROR_CHECK(esp_zb_humidity_meas_cluster_add_attr(hum_attr_list, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_TOLERANCE_ID, &s_fake_humi_tolerance_centi_pct));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(sensor_clusters, hum_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_endpoint_config_t sensor_ep_cfg = {
        .endpoint = HA_TEMP_HUMI_SENSOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(ep_list, sensor_clusters, sensor_ep_cfg));

    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };

    esp_zcl_utility_add_ep_basic_manufacturer_info(ep_list, HA_ONOFF_SWITCH_ENDPOINT, &info);
    esp_zcl_utility_add_ep_basic_manufacturer_info(ep_list, HA_ONOFF_RELAY_ENDPOINT, &info);
    esp_zcl_utility_add_ep_basic_manufacturer_info(ep_list, HA_TEMP_HUMI_SENSOR_ENDPOINT, &info);

    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
