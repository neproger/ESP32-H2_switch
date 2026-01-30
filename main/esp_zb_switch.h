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
#define HA_ONOFF_SWITCH_ENDPOINT        1          /* Zigbee On/Off switch (client) endpoint */
#define HA_ONOFF_RELAY_ENDPOINT         2          /* Zigbee On/Off light (server) endpoint (test relay) */
#define HA_TEMP_HUMI_SENSOR_ENDPOINT    3          /* Zigbee Temperature + Humidity sensor endpoint */
#define HA_RGB_LIGHT_ENDPOINT           4          /* Zigbee Color Dimmable Light (server) endpoint */

/* Board GPIOs (adjust for your ESP32-H2 board if needed) */
/* Test "relay" output (some boards have a user LED on GPIO13). */
#define GPIO_OUTPUT_IO_RELAY_TEST       GPIO_NUM_13
/* Set to 0 if your LED/relay is active-low (GPIO low = ON). */
#define GPIO_OUTPUT_IO_RELAY_TEST_ACTIVE_LEVEL 1

/* On-board RGB LED data pin (often WS2812/NeoPixel on GPIO8). */
#define GPIO_OUTPUT_IO_RGB_LED_DATA     GPIO_NUM_8

/* Let the device join any 2.4GHz Zigbee channel (11-26). */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

#define INSTALLCODE_POLICY_ENABLE       false      /* enable the install code policy for security */

/* Router configuration (mains-powered) */
#define ESP_ZB_MAX_CHILDREN             10U

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x09""ESPRESSIF"      /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x07"CONFIG_IDF_TARGET /* Customized model identifier */

#define ESP_ZB_ZR_CONFIG()                                                              \
    {                                                                                   \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,                                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,                               \
        .nwk_cfg.zczr_cfg = {                                                           \
            .max_children = ESP_ZB_MAX_CHILDREN,                                        \
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
