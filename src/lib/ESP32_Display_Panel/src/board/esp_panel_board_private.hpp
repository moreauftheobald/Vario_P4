/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @note This file shouldn't be included in the public header file.
 */
#pragma once

// *INDENT-OFF*
#include "esp_panel_board_conf_internal.h"
#include "custom/esp_panel_board_config_custom.h"

/* Check if select both custom and supported board */
#if ESP_PANEL_BOARD_DEFAULT_USE_SUPPORTED && ESP_PANEL_BOARD_DEFAULT_USE_CUSTOM
#error "Please select either a custom or a supported development board, cannot enable both simultaneously"
#endif

/* Check if using a default board */
#define ESP_PANEL_BOARD_USE_DEFAULT     (ESP_PANEL_BOARD_DEFAULT_USE_SUPPORTED || ESP_PANEL_BOARD_DEFAULT_USE_CUSTOM)

#if ESP_PANEL_BOARD_USE_DEFAULT
/**
 * There are three purposes to include the this file:
 *  1. Convert configuration items starting with `CONFIG_` to the required configuration items.
 *  2. Define default values for configuration items that are not defined to keep compatibility.
 *  3. Check if missing configuration items
 */
#include "custom/esp_panel_board_kconfig_custom.h"
#endif

/**
 * Define the name of drivers
 */
#ifdef ESP_PANEL_BOARD_LCD_BUS_TYPE
#if ESP_PANEL_BOARD_LCD_BUS_TYPE == ESP_PANEL_BUS_TYPE_MIPI_DSI
#define ESP_PANEL_BOARD_LCD_BUS_NAME    DSI
#else
#error "Unknown LCD bus type selected! Only MIPI DSI is supported."
#endif
#endif

#ifdef ESP_PANEL_BOARD_BACKLIGHT_TYPE
#if ESP_PANEL_BOARD_BACKLIGHT_TYPE == ESP_PANEL_BACKLIGHT_TYPE_PWM_LEDC
#define ESP_PANEL_BOARD_BACKLIGHT_NAME    PWM_LEDC
#else
#error "Unknown backlight type selected! Only PWM_LEDC is supported."
#endif
#endif

// *INDENT-ON*
