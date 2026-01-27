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
#include "esp_zigbee_core.h"
#include "switch_driver.h"
#include "zcl_utility.h"

/* Zigbee configuration */
#define HA_ONOFF_SWITCH_ENDPOINT        1          /* esp light switch device endpoint */
/* Let the device join any 2.4GHz Zigbee channel (11-26). */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

#define INSTALLCODE_POLICY_ENABLE       false      /* enable the install code policy for security */
/* End-device keep-alive must be lower than the parent aging timeout. */
#define ESP_ZB_ED_TIMEOUT               ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ESP_ZB_ED_KEEP_ALIVE_MS         3000U

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x09""ESPRESSIF"      /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x07"CONFIG_IDF_TARGET /* Customized model identifier */

#define ESP_ZB_ZED_CONFIG()                                                             \
    {                                                                                   \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                                           \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,                               \
        .nwk_cfg.zed_cfg = {                                                            \
            .ed_timeout = ESP_ZB_ED_TIMEOUT,                                            \
            .keep_alive = ESP_ZB_ED_KEEP_ALIVE_MS,                                      \
        },                                                                              \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }
