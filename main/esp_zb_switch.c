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
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"
#include "esp_zigbee_endpoint.h"
#include "zcl/esp_zigbee_zcl_humidity_meas.h"
#include "zcl/esp_zigbee_zcl_level.h"
#include "zcl/esp_zigbee_zcl_color_control.h"
#include "nwk/esp_zigbee_nwk.h"
#include "zdo/esp_zigbee_zdo_command.h"
#include "zboss_api.h"
#include "esp_zb_switch.h"
#include <stdlib.h>
#include <math.h>

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

/* RGB LED (WS2812/NeoPixel) state for HA_RGB_LIGHT_ENDPOINT */
static bool s_rgb_on;
static uint8_t s_rgb_level = ESP_ZB_ZCL_LEVEL_CONTROL_CURRENT_LEVEL_DEFAULT_VALUE;
static uint8_t s_rgb_hue = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_HUE_DEFAULT_VALUE;
static uint8_t s_rgb_sat = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_SATURATION_DEFAULT_VALUE;
static uint16_t s_rgb_x = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_X_DEF_VALUE;
static uint16_t s_rgb_y = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_Y_DEF_VALUE;
static uint8_t s_rgb_color_mode = ESP_ZB_ZCL_COLOR_CONTROL_COLOR_MODE_DEFAULT_VALUE;

static rmt_channel_handle_t s_rgb_rmt_chan;
static rmt_encoder_handle_t s_rgb_rmt_encoder;

typedef struct {
    uint32_t version;
    uint8_t on;
    uint8_t level;
    uint8_t hue;
    uint8_t sat;
    uint8_t color_mode;
    uint16_t x;
    uint16_t y;
} rgb_persist_t;

static bool s_rgb_persist_loaded;
static esp_timer_handle_t s_rgb_persist_timer;

static bool s_rgb_identify_active;
static esp_timer_handle_t s_rgb_identify_timer;
static rgb_persist_t s_rgb_identify_saved;

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

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} rmt_ws2812_encoder_t;

static size_t rmt_encode_ws2812(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data,
                                size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_ws2812_encoder_t *ws = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (ws->state) {
    case 0:
        encoded_symbols += ws->bytes_encoder->encode(ws->bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        /* fallthrough */
    case 1:
        encoded_symbols += ws->copy_encoder->encode(ws->copy_encoder, channel, &ws->reset_code, sizeof(ws->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            ws->state = 0;
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        break;
    default:
        break;
    }

out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_ws2812_encoder(rmt_encoder_t *encoder)
{
    rmt_ws2812_encoder_t *ws = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_del_encoder(ws->bytes_encoder);
    rmt_del_encoder(ws->copy_encoder);
    free(ws);
    return ESP_OK;
}

static esp_err_t rmt_reset_ws2812_encoder(rmt_encoder_t *encoder)
{
    rmt_ws2812_encoder_t *ws = __containerof(encoder, rmt_ws2812_encoder_t, base);
    rmt_encoder_reset(ws->bytes_encoder);
    rmt_encoder_reset(ws->copy_encoder);
    ws->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t rgb_led_new_ws2812_encoder(rmt_encoder_handle_t *ret_encoder)
{
    rmt_ws2812_encoder_t *ws = (rmt_ws2812_encoder_t *)calloc(1, sizeof(rmt_ws2812_encoder_t));
    ESP_RETURN_ON_FALSE(ws, ESP_ERR_NO_MEM, TAG, "ws2812 encoder: OOM");

    ws->base.encode = rmt_encode_ws2812;
    ws->base.del = rmt_del_ws2812_encoder;
    ws->base.reset = rmt_reset_ws2812_encoder;

    /* WS2812 @ 10MHz resolution (1 tick = 0.1us). Data is GRB, MSB first. */
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 3, /* T0H=0.3us */
            .level1 = 0,
            .duration1 = 9, /* T0L=0.9us */
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 9, /* T1H=0.9us */
            .level1 = 0,
            .duration1 = 3, /* T1L=0.3us */
        },
        .flags.msb_first = 1,
    };
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &ws->bytes_encoder), TAG, "ws2812 bytes encoder");

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &ws->copy_encoder), TAG, "ws2812 copy encoder");

    ws->reset_code = (rmt_symbol_word_t){
        .level0 = 0,
        .duration0 = 250,
        .level1 = 0,
        .duration1 = 250,
    }; /* 50us reset */

    *ret_encoder = &ws->base;
    return ESP_OK;
}

static uint8_t rgb_gamma8(float c)
{
    if (c <= 0.0f) {
        return 0;
    }
    if (c >= 1.0f) {
        return 255;
    }
    /* sRGB gamma */
    float v = (c <= 0.0031308f) ? (12.92f * c) : (1.055f * powf(c, 1.0f / 2.4f) - 0.055f);
    if (v < 0.0f) {
        v = 0.0f;
    } else if (v > 1.0f) {
        v = 1.0f;
    }
    return (uint8_t)lroundf(v * 255.0f);
}

static void rgb_xy_to_rgb8(uint16_t x16, uint16_t y16, uint8_t level, uint8_t *r8, uint8_t *g8, uint8_t *b8)
{
    /* Convert CIE 1931 xy to sRGB (approx). */
    const float x = (float)x16 / 65535.0f;
    const float y = (float)y16 / 65535.0f;
    const float bri = (float)level / 254.0f;

    if (y <= 0.0001f || bri <= 0.0f) {
        *r8 = *g8 = *b8 = 0;
        return;
    }

    const float Y = 1.0f;
    const float X = (Y / y) * x;
    const float Z = (Y / y) * (1.0f - x - y);

    float r = 3.2406f * X - 1.5372f * Y - 0.4986f * Z;
    float g = -0.9689f * X + 1.8758f * Y + 0.0415f * Z;
    float b = 0.0557f * X - 0.2040f * Y + 1.0570f * Z;

    if (r < 0.0f) {
        r = 0.0f;
    }
    if (g < 0.0f) {
        g = 0.0f;
    }
    if (b < 0.0f) {
        b = 0.0f;
    }

    float maxc = r;
    if (g > maxc) {
        maxc = g;
    }
    if (b > maxc) {
        maxc = b;
    }
    if (maxc > 1.0f) {
        r /= maxc;
        g /= maxc;
        b /= maxc;
    }

    r *= bri;
    g *= bri;
    b *= bri;

    *r8 = rgb_gamma8(r);
    *g8 = rgb_gamma8(g);
    *b8 = rgb_gamma8(b);
}

static void rgb_hs_to_rgb8(uint8_t hue, uint8_t sat, uint8_t level, uint8_t *r8, uint8_t *g8, uint8_t *b8)
{
    const float h = ((float)hue / 254.0f) * 360.0f;
    const float s = (float)sat / 254.0f;
    const float v = (float)level / 254.0f;

    if (v <= 0.0f) {
        *r8 = *g8 = *b8 = 0;
        return;
    }

    const float c = v * s;
    const float hh = fmodf(h, 360.0f) / 60.0f;
    const float x = c * (1.0f - fabsf(fmodf(hh, 2.0f) - 1.0f));
    float r1 = 0.0f, g1 = 0.0f, b1 = 0.0f;

    if (hh < 1.0f) {
        r1 = c;
        g1 = x;
    } else if (hh < 2.0f) {
        r1 = x;
        g1 = c;
    } else if (hh < 3.0f) {
        g1 = c;
        b1 = x;
    } else if (hh < 4.0f) {
        g1 = x;
        b1 = c;
    } else if (hh < 5.0f) {
        r1 = x;
        b1 = c;
    } else {
        r1 = c;
        b1 = x;
    }

    const float m = v - c;
    *r8 = rgb_gamma8(r1 + m);
    *g8 = rgb_gamma8(g1 + m);
    *b8 = rgb_gamma8(b1 + m);
}

static void rgb_led_send(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_rgb_rmt_chan || !s_rgb_rmt_encoder) {
        return;
    }
    /* WS2812 expects GRB order */
    uint8_t grb[3] = {g, r, b};
    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };
    if (rmt_transmit(s_rgb_rmt_chan, s_rgb_rmt_encoder, grb, sizeof(grb), &tx_cfg) == ESP_OK) {
        (void)rmt_tx_wait_all_done(s_rgb_rmt_chan, portMAX_DELAY);
    }
}

static void rgb_led_apply(void)
{
    if (s_rgb_identify_active) {
        return;
    }
    if (!s_rgb_on || s_rgb_level == 0) {
        rgb_led_send(0, 0, 0);
        return;
    }
    uint8_t r = 0, g = 0, b = 0;
    if (s_rgb_color_mode == 0x00) { /* Hue/Sat */
        rgb_hs_to_rgb8(s_rgb_hue, s_rgb_sat, s_rgb_level, &r, &g, &b);
    } else { /* XY (default) */
        rgb_xy_to_rgb8(s_rgb_x, s_rgb_y, s_rgb_level, &r, &g, &b);
    }
    rgb_led_send(r, g, b);
}

static rgb_persist_t rgb_state_snapshot(void)
{
    rgb_persist_t st = {
        .version = 1,
        .on = s_rgb_on ? 1 : 0,
        .level = s_rgb_level,
        .hue = s_rgb_hue,
        .sat = s_rgb_sat,
        .color_mode = s_rgb_color_mode,
        .x = s_rgb_x,
        .y = s_rgb_y,
    };
    return st;
}

static void rgb_state_apply_snapshot(const rgb_persist_t *st)
{
    if (!st) {
        return;
    }
    s_rgb_on = st->on ? true : false;
    s_rgb_level = st->level;
    s_rgb_hue = st->hue;
    s_rgb_sat = st->sat;
    s_rgb_color_mode = st->color_mode;
    s_rgb_x = st->x;
    s_rgb_y = st->y;
}

static void rgb_state_save_now(void)
{
    nvs_handle_t h = 0;
    if (nvs_open("rgb", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    const rgb_persist_t st = rgb_state_snapshot();
    (void)nvs_set_blob(h, "state", &st, sizeof(st));
    (void)nvs_commit(h);
    nvs_close(h);
}

static void rgb_persist_timer_cb(void *arg)
{
    (void)arg;
    rgb_state_save_now();
}

static void rgb_state_request_save(void)
{
    if (!s_rgb_persist_timer) {
        const esp_timer_create_args_t t = {
            .callback = rgb_persist_timer_cb,
            .name = "rgb_save",
        };
        if (esp_timer_create(&t, &s_rgb_persist_timer) != ESP_OK) {
            return;
        }
    }
    (void)esp_timer_stop(s_rgb_persist_timer);
    (void)esp_timer_start_once(s_rgb_persist_timer, 500 * 1000); /* 500ms debounce */
}

static void rgb_state_load_from_nvs(void)
{
    nvs_handle_t h = 0;
    if (nvs_open("rgb", NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    rgb_persist_t st = {0};
    size_t len = sizeof(st);
    if (nvs_get_blob(h, "state", &st, &len) == ESP_OK && len == sizeof(st) && st.version == 1) {
        rgb_state_apply_snapshot(&st);
        s_rgb_persist_loaded = true;
    }
    nvs_close(h);
}

static esp_err_t rgb_led_init(void)
{
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = GPIO_OUTPUT_IO_RGB_LED_DATA,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, /* 10MHz */
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .flags.invert_out = 0,
        .flags.with_dma = 0,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_cfg, &s_rgb_rmt_chan), TAG, "rgb rmt_new_tx_channel");
    ESP_RETURN_ON_ERROR(rgb_led_new_ws2812_encoder(&s_rgb_rmt_encoder), TAG, "rgb ws2812 encoder");
    ESP_RETURN_ON_ERROR(rmt_enable(s_rgb_rmt_chan), TAG, "rgb rmt_enable");
    rgb_led_send(0, 0, 0);
    rgb_led_apply();
    return ESP_OK;
}

static const char *zb_zcl_cmd_direction_to_string(uint8_t dir)
{
    return dir ? "to_cli" : "to_srv";
}

static const char *zb_on_off_cmd_to_string(uint8_t cmd_id)
{
    switch (cmd_id) {
    case 0x00:
        return "OFF";
    case 0x01:
        return "ON";
    case 0x02:
        return "TOGGLE";
    case 0x40:
        return "OFF_WITH_EFFECT";
    case 0x41:
        return "ON_WITH_RECALL_GLOBAL_SCENE";
    case 0x42:
        return "ON_WITH_TIMED_OFF";
    default:
        return "UNKNOWN";
    }
}

static void zb_format_zboss_source(char *out, size_t out_len, const zb_zcl_addr_t *src)
{
    if (!src) {
        (void)snprintf(out, out_len, "src=<null>");
        return;
    }
    switch (src->addr_type) {
    case ZB_ZCL_ADDR_TYPE_SHORT:
        (void)snprintf(out, out_len, "short=0x%04hx", src->u.short_addr);
        return;
    case ZB_ZCL_ADDR_TYPE_IEEE: {
        /* ZBOSS stores IEEE in little-endian order (same as esp_zb_ieee_addr_t). */
        char ieee[24] = {0};
        zb_format_ieee_addr(ieee, sizeof(ieee), (const uint8_t *)src->u.ieee_addr);
        (void)snprintf(out, out_len, "ieee=%s", ieee);
        return;
    }
    default:
        (void)snprintf(out, out_len, "addr_type=%u", (unsigned)src->addr_type);
        return;
    }
}

static uint16_t zb_u16_le(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static size_t zb_zcl_header_len_from_fc(uint8_t fc)
{
    /* ZCL Frame Control: bit2 = manufacturer specific */
    return (fc & 0x04) ? 5 : 3;
}

static void rgb_handle_incoming_zcl_command(const zb_zcl_parsed_hdr_t *hdr, const uint8_t *raw, uint16_t raw_len)
{
    if (!hdr || !raw || raw_len < 3) {
        return;
    }
    const uint8_t dst_ep = ZB_ZCL_PARSED_HDR_SHORT_DATA(hdr).dst_endpoint;
    if (dst_ep != HA_RGB_LIGHT_ENDPOINT) {
        return;
    }
    if (hdr->is_common_command) {
        return;
    }

    const size_t zcl_hdr_len = zb_zcl_header_len_from_fc(raw[0]);
    if (raw_len < zcl_hdr_len) {
        return;
    }
    const uint8_t *pl = raw + zcl_hdr_len;
    const uint16_t pl_len = (uint16_t)(raw_len - zcl_hdr_len);

    bool changed = false;

    if (hdr->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL) {
        if ((hdr->cmd_id == ESP_ZB_ZCL_CMD_LEVEL_CONTROL_MOVE_TO_LEVEL ||
             hdr->cmd_id == ESP_ZB_ZCL_CMD_LEVEL_CONTROL_MOVE_TO_LEVEL_WITH_ON_OFF) &&
            pl_len >= 3) {
            s_rgb_level = pl[0];
            changed = true;
        }
    } else if (hdr->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL) {
        if (hdr->cmd_id == ESP_ZB_ZCL_CMD_COLOR_CONTROL_MOVE_TO_COLOR && pl_len >= 6) {
            s_rgb_x = zb_u16_le(&pl[0]);
            s_rgb_y = zb_u16_le(&pl[2]);
            s_rgb_color_mode = 0x01; /* XY */
            changed = true;
        } else if (hdr->cmd_id == ESP_ZB_ZCL_CMD_COLOR_CONTROL_MOVE_TO_HUE_SATURATION && pl_len >= 4) {
            s_rgb_hue = pl[0];
            s_rgb_sat = pl[1];
            s_rgb_color_mode = 0x00; /* Hue/Sat */
            changed = true;
        } else if (hdr->cmd_id == ESP_ZB_ZCL_CMD_COLOR_CONTROL_MOVE_TO_HUE && pl_len >= 4) {
            s_rgb_hue = pl[0];
            s_rgb_color_mode = 0x00; /* Hue/Sat */
            changed = true;
        }
    } else if (hdr->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        if (hdr->cmd_id == 0x00) { /* OFF */
            s_rgb_on = false;
            changed = true;
        } else if (hdr->cmd_id == 0x01) { /* ON */
            s_rgb_on = true;
            changed = true;
        } else if (hdr->cmd_id == 0x02) { /* TOGGLE */
            s_rgb_on = !s_rgb_on;
            changed = true;
        }
    }

    if (changed) {
        rgb_led_apply();
        rgb_state_request_save();
    }
}

static bool zb_device_cb_id_logger(uint8_t bufid)
{
    const zb_zcl_device_callback_param_t *p = ZB_ZCL_DEVICE_CMD_PARAM(bufid);
    if (!p) {
        return false;
    }

    const zb_zcl_parsed_hdr_t *hdr = p->cb_param.gnr.in_cmd_info;
    if (!hdr) {
        return false;
    }

    const uint8_t *raw = (const uint8_t *)zb_buf_begin(bufid);
    const uint16_t raw_len = (uint16_t)zb_buf_len(bufid);

    /* For RGB light, many gateways use commands (MoveToColor/MoveToLevel...) rather than WriteAttributes.
     * Capture those even when the light is OFF so turning it ON later uses the latest selected color/level. */
    rgb_handle_incoming_zcl_command(hdr, raw, raw_len);

    /* Reduce noise: focus on commands hitting our "relay" endpoint, RGB endpoint, and/or On/Off cluster. */
    const uint8_t dst_ep = ZB_ZCL_PARSED_HDR_SHORT_DATA(hdr).dst_endpoint;
    if (dst_ep != HA_ONOFF_RELAY_ENDPOINT && dst_ep != HA_RGB_LIGHT_ENDPOINT &&
        hdr->cluster_id != ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        return false;
    }

    char src_str[48] = {0};
    zb_format_zboss_source(src_str, sizeof(src_str), &ZB_ZCL_PARSED_HDR_SHORT_DATA(hdr).source);

    const char *cmd_name = (hdr->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF && !hdr->is_common_command)
                               ? zb_on_off_cmd_to_string(hdr->cmd_id)
                               : (hdr->is_common_command ? "ZCL_COMMON" : "CLUSTER_SPECIFIC");

    ESP_LOGI(TAG,
             "RX ZCL: cb_id=0x%04hx status=%d ep=%u %s -> dst_short=0x%04hx src_ep=%u dst_ep=%u profile=0x%04hx cluster=0x%04hx cmd=%s(0x%02x) dir=%s tsn=%u common=%u manuf=%u/0x%04hx",
             (uint16_t)p->device_cb_id, (int)p->status, (unsigned)p->endpoint, src_str,
             ZB_ZCL_PARSED_HDR_SHORT_DATA(hdr).dst_addr,
             (unsigned)ZB_ZCL_PARSED_HDR_SHORT_DATA(hdr).src_endpoint, (unsigned)dst_ep,
             hdr->profile_id, hdr->cluster_id, cmd_name, (unsigned)hdr->cmd_id,
             zb_zcl_cmd_direction_to_string(hdr->cmd_direction), (unsigned)hdr->seq_number,
             (unsigned)hdr->is_common_command, (unsigned)hdr->is_manuf_specific, hdr->manuf_specific);

    if (raw && raw_len) {
        const uint16_t dump_len = raw_len > 64 ? 64 : raw_len;
        ESP_LOG_BUFFER_HEXDUMP(TAG, raw, dump_len, ESP_LOG_INFO);
    }

    /* Do not consume the buffer; allow stack to continue default processing. */
    return false;
}

static void rgb_identify_timer_cb(void *arg)
{
    (void)arg;
    static uint8_t anim_hue = 0;
    anim_hue = (uint8_t)(anim_hue + 3);
    uint8_t r = 0, g = 0, b = 0;
    rgb_hs_to_rgb8(anim_hue, 254, 180, &r, &g, &b);
    rgb_led_send(r, g, b);
}

static void rgb_identify_notify_cb(uint8_t identify_on)
{
    if (identify_on) {
        if (!s_rgb_identify_active) {
            s_rgb_identify_saved = rgb_state_snapshot();
        }
        s_rgb_identify_active = true;

        if (!s_rgb_identify_timer) {
            const esp_timer_create_args_t t = {
                .callback = rgb_identify_timer_cb,
                .name = "rgb_ident",
            };
            if (esp_timer_create(&t, &s_rgb_identify_timer) != ESP_OK) {
                return;
            }
        }
        (void)esp_timer_stop(s_rgb_identify_timer);
        (void)esp_timer_start_periodic(s_rgb_identify_timer, 40 * 1000); /* 25Hz */
    } else {
        s_rgb_identify_active = false;
        if (s_rgb_identify_timer) {
            (void)esp_timer_stop(s_rgb_identify_timer);
        }
        rgb_state_apply_snapshot(&s_rgb_identify_saved);
        rgb_led_apply();
    }
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
    gpio_set_level(GPIO_OUTPUT_IO_RELAY_TEST, !GPIO_OUTPUT_IO_RELAY_TEST_ACTIVE_LEVEL);
    s_relay_state = false;
}

static void relay_set(bool on)
{
    s_relay_state = on;
    gpio_set_level(GPIO_OUTPUT_IO_RELAY_TEST, on ? GPIO_OUTPUT_IO_RELAY_TEST_ACTIVE_LEVEL
                                                : !GPIO_OUTPUT_IO_RELAY_TEST_ACTIVE_LEVEL);
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

    if (message->info.dst_endpoint == HA_RGB_LIGHT_ENDPOINT) {
        bool updated = false;
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF &&
            message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
            message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
            s_rgb_on = message->attribute.data.value ? *(bool *)message->attribute.data.value : false;
            rgb_led_apply();
            updated = true;
        } else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL &&
                   message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID &&
                   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
            s_rgb_level = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : s_rgb_level;
            rgb_led_apply();
            updated = true;
        } else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL &&
                   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
                   message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID) {
            s_rgb_x = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : s_rgb_x;
            rgb_led_apply();
            updated = true;
        } else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL &&
                   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16 &&
                   message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID) {
            s_rgb_y = message->attribute.data.value ? *(uint16_t *)message->attribute.data.value : s_rgb_y;
            rgb_led_apply();
            updated = true;
        } else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL &&
                   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8 &&
                   message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_MODE_ID) {
            s_rgb_color_mode = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : s_rgb_color_mode;
            rgb_led_apply();
            updated = true;
        } else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL &&
                   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8 &&
                   message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID) {
            s_rgb_hue = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : s_rgb_hue;
            rgb_led_apply();
            updated = true;
        } else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL &&
                   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8 &&
                   message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_SATURATION_ID) {
            s_rgb_sat = message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : s_rgb_sat;
            rgb_led_apply();
            updated = true;
        }

        if (updated) {
            rgb_state_request_save();
        }
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
    ESP_RETURN_ON_ERROR(rgb_led_init(), TAG, "Failed to init RGB LED");

    if (s_rgb_persist_loaded) {
        esp_zb_lock_acquire(portMAX_DELAY);
        (void)esp_zb_zcl_set_attribute_val(HA_RGB_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                                          &s_rgb_on, false);
        (void)esp_zb_zcl_set_attribute_val(HA_RGB_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
                                          &s_rgb_level, false);
        (void)esp_zb_zcl_set_attribute_val(HA_RGB_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID,
                                          &s_rgb_x, false);
        (void)esp_zb_zcl_set_attribute_val(HA_RGB_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID,
                                          &s_rgb_y, false);
        (void)esp_zb_zcl_set_attribute_val(HA_RGB_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_MODE_ID,
                                          &s_rgb_color_mode, false);
        (void)esp_zb_zcl_set_attribute_val(HA_RGB_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID,
                                          &s_rgb_hue, false);
        (void)esp_zb_zcl_set_attribute_val(HA_RGB_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL,
                                          ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_SATURATION_ID,
                                          &s_rgb_sat, false);
        esp_zb_lock_release();
    }
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

    /* Endpoint 4: Color dimmable light (server) for on-board RGB LED */
    esp_zb_color_dimmable_light_cfg_t rgb_cfg = ESP_ZB_DEFAULT_COLOR_DIMMABLE_LIGHT_CONFIG();
    /* Advertise Hue/Sat + XY so most gateways can pick their preferred mode. */
    rgb_cfg.color_cfg.color_capabilities = 0x0009;
    esp_zb_cluster_list_t *rgb_clusters = esp_zb_color_dimmable_light_clusters_create(&rgb_cfg);
    esp_zb_endpoint_config_t rgb_ep_cfg = {
        .endpoint = HA_RGB_LIGHT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID,
        .app_device_version = 0,
    };
    ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(ep_list, rgb_clusters, rgb_ep_cfg));
    esp_zcl_utility_add_ep_basic_manufacturer_info(ep_list, HA_RGB_LIGHT_ENDPOINT, &info);

    esp_zb_device_register(ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_device_cb_id_handler_register(zb_device_cb_id_logger);
    esp_zb_identify_notify_handler_register(HA_RGB_LIGHT_ENDPOINT, rgb_identify_notify_cb);
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
    rgb_state_load_from_nvs();
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
