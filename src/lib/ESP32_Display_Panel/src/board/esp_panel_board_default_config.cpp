/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "esp_panel_types.h"
#include "utils/esp_panel_utils_log.h"
#include "utils/esp_panel_utils_cxx.hpp"
#include "board/esp_panel_board_config.hpp"
#include "board/esp_panel_board.hpp"
// Replace the following header file if creating a new board configuration
#include "board/esp_panel_board_private.hpp"
#include "esp_panel_board_default_config.hpp"

// *INDENT-OFF*
#undef _TO_STR
#undef TO_STR
#define _TO_STR(name) #name
#define TO_STR(name) _TO_STR(name)

using namespace esp_panel::drivers;
using namespace esp_panel::board;

#ifdef ESP_PANEL_BOARD_LCD_VENDOR_INIT_CMD
static const esp_panel_lcd_vendor_init_cmd_t lcd_vendor_init_cmds[] = ESP_PANEL_BOARD_LCD_VENDOR_INIT_CMD();
#endif // ESP_PANEL_BOARD_LCD_VENDOR_INIT_CMD

const BoardConfig ESP_PANEL_BOARD_DEFAULT_CONFIG = {
    /* General */
    #ifdef ESP_PANEL_BOARD_NAME
    .name = ESP_PANEL_BOARD_NAME,
    #endif

    /* LCD */
    #if ESP_PANEL_BOARD_USE_LCD
    .lcd = BoardConfig::LCD_Config{
        #if (ESP_PANEL_BOARD_LCD_BUS_TYPE == ESP_PANEL_BUS_TYPE_MIPI_DSI) && ESP_PANEL_DRIVERS_BUS_ENABLE_MIPI_DSI
        .bus_config = BusDSI::Config{
            // Host
            .host = BusDSI::HostPartialConfig{
                .num_data_lanes = ESP_PANEL_BOARD_LCD_MIPI_DSI_LANE_NUM,
                .lane_bit_rate_mbps = ESP_PANEL_BOARD_LCD_MIPI_DSI_LANE_RATE_MBPS,
            },
            // Panel
            .refresh_panel = BusDSI::RefreshPanelPartialConfig{
                .dpi_clock_freq_mhz = ESP_PANEL_BOARD_LCD_MIPI_DPI_CLK_MHZ,
                .bits_per_pixel = ESP_PANEL_BOARD_LCD_MIPI_DPI_PIXEL_BITS,
                .h_size = ESP_PANEL_BOARD_WIDTH,
                .v_size = ESP_PANEL_BOARD_HEIGHT,
                .hsync_pulse_width = ESP_PANEL_BOARD_LCD_MIPI_DPI_HPW,
                .hsync_back_porch = ESP_PANEL_BOARD_LCD_MIPI_DPI_HBP,
                .hsync_front_porch = ESP_PANEL_BOARD_LCD_MIPI_DPI_HFP,
                .vsync_pulse_width = ESP_PANEL_BOARD_LCD_MIPI_DPI_VPW,
                .vsync_back_porch = ESP_PANEL_BOARD_LCD_MIPI_DPI_VBP,
                .vsync_front_porch = ESP_PANEL_BOARD_LCD_MIPI_DPI_VFP,
            },
            // PHY LDO
            .phy_ldo = BusDSI::PHY_LDO_PartialConfig{
                .chan_id = ESP_PANEL_BOARD_LCD_MIPI_PHY_LDO_ID
            },
        },
        #endif // ESP_PANEL_BOARD_LCD_BUS_TYPE
        .device_name = TO_STR(ESP_PANEL_BOARD_LCD_CONTROLLER),
        .device_config = {
            // Device
            .device = LCD::DevicePartialConfig{
                .reset_gpio_num = ESP_PANEL_BOARD_LCD_RST_IO,
                .rgb_ele_order = ESP_PANEL_BOARD_LCD_COLOR_BGR_ORDER,
                .bits_per_pixel = ESP_PANEL_BOARD_LCD_COLOR_BITS,
                .flags_reset_active_high = ESP_PANEL_BOARD_LCD_RST_LEVEL,
            },
            // Vendor
            .vendor = LCD::VendorPartialConfig{
                .hor_res = ESP_PANEL_BOARD_WIDTH,
                .ver_res = ESP_PANEL_BOARD_HEIGHT,
                #ifdef ESP_PANEL_BOARD_LCD_VENDOR_INIT_CMD
                .init_cmds = lcd_vendor_init_cmds,
                .init_cmds_size = sizeof(lcd_vendor_init_cmds) / sizeof(lcd_vendor_init_cmds[0]),
                #endif // ESP_PANEL_BOARD_LCD_VENDOR_INIT_CMD
                #ifdef ESP_PANEL_BOARD_LCD_FLAGS_MIRROR_BY_CMD
                .flags_mirror_by_cmd = ESP_PANEL_BOARD_LCD_FLAGS_MIRROR_BY_CMD,
                #endif // ESP_PANEL_BOARD_LCD_FLAGS_MIRROR_BY_CMD
                #ifdef ESP_PANEL_BOARD_LCD_FLAGS_ENABLE_IO_MULTIPLEX
                .flags_enable_io_multiplex = ESP_PANEL_BOARD_LCD_FLAGS_ENABLE_IO_MULTIPLEX,
                #endif // ESP_PANEL_BOARD_LCD_FLAGS_ENABLE_IO_MULTIPLEX
            },
        },
        .pre_process = {
            .invert_color = ESP_PANEL_BOARD_LCD_COLOR_INEVRT_BIT,
            #ifdef ESP_PANEL_BOARD_LCD_SWAP_XY
            .swap_xy = ESP_PANEL_BOARD_LCD_SWAP_XY,
            #endif // ESP_PANEL_BOARD_LCD_SWAP_XY
            #ifdef ESP_PANEL_BOARD_LCD_MIRROR_X
            .mirror_x = ESP_PANEL_BOARD_LCD_MIRROR_X,
            #endif // ESP_PANEL_BOARD_LCD_MIRROR_X
            #ifdef ESP_PANEL_BOARD_LCD_MIRROR_Y
            .mirror_y = ESP_PANEL_BOARD_LCD_MIRROR_Y,
            #endif // ESP_PANEL_BOARD_LCD_MIRROR_Y
            #ifdef ESP_PANEL_BOARD_LCD_GAP_X
            .gap_x = ESP_PANEL_BOARD_LCD_GAP_X,
            #endif // ESP_PANEL_BOARD_LCD_GAP_X
            #ifdef ESP_PANEL_BOARD_LCD_GAP_Y
            .gap_y = ESP_PANEL_BOARD_LCD_GAP_Y,
            #endif // ESP_PANEL_BOARD_LCD_GAP_Y
        },
    },
    #endif // ESP_PANEL_BOARD_USE_LCD

    /* Backlight */
    #if ESP_PANEL_BOARD_USE_BACKLIGHT
    .backlight = BoardConfig::BacklightConfig{
        #if ESP_PANEL_BOARD_BACKLIGHT_TYPE == ESP_PANEL_BACKLIGHT_TYPE_PWM_LEDC
        .config = BacklightPWM_LEDC::Config{
            .ledc_timer = BacklightPWM_LEDC::LEDC_TimerPartialConfig{
                .freq_hz = ESP_PANEL_BOARD_BACKLIGHT_PWM_FREQ_HZ,
                .duty_resolution = ESP_PANEL_BOARD_BACKLIGHT_PWM_DUTY_RESOLUTION,
            },
            .ledc_channel = BacklightPWM_LEDC::LEDC_ChannelPartialConfig{
                .io_num = ESP_PANEL_BOARD_BACKLIGHT_IO,
                .on_level = ESP_PANEL_BOARD_BACKLIGHT_ON_LEVEL,
            },
        },
        #endif // ESP_PANEL_BOARD_BACKLIGHT_TYPE
        .pre_process = {
            .idle_off = ESP_PANEL_BOARD_BACKLIGHT_IDLE_OFF,
        },
    },
    #endif // ESP_PANEL_BOARD_USE_BACKLIGHT

    /* Others */
    .stage_callbacks = {
        #ifdef ESP_PANEL_BOARD_PRE_BEGIN_FUNCTION
        [](void *p) -> bool ESP_PANEL_BOARD_PRE_BEGIN_FUNCTION(p),
        #else
        nullptr,
        #endif // ESP_PANEL_BOARD_PRE_BEGIN_FUNCTION
        #ifdef ESP_PANEL_BOARD_POST_BEGIN_FUNCTION
        [](void *p) -> bool ESP_PANEL_BOARD_POST_BEGIN_FUNCTION(p),
        #else
        nullptr,
        #endif // ESP_PANEL_BOARD_POST_BEGIN_FUNCTION
        #ifdef ESP_PANEL_BOARD_PRE_DEL_FUNCTION
        [](void *p) -> bool ESP_PANEL_BOARD_PRE_DEL_FUNCTION(p),
        #else
        nullptr,
        #endif // ESP_PANEL_BOARD_PRE_DEL_FUNCTION
        #ifdef ESP_PANEL_BOARD_POST_DEL_FUNCTION
        [](void *p) -> bool ESP_PANEL_BOARD_POST_DEL_FUNCTION(p),
        #else
        nullptr,
        #endif // ESP_PANEL_BOARD_POST_DEL_FUNCTION
        #ifdef ESP_PANEL_BOARD_LCD_PRE_BEGIN_FUNCTION
        [](void *p) -> bool ESP_PANEL_BOARD_LCD_PRE_BEGIN_FUNCTION(p),
        #else
        nullptr,
        #endif // ESP_PANEL_BOARD_LCD_PRE_BEGIN_FUNCTION
        #ifdef ESP_PANEL_BOARD_LCD_POST_BEGIN_FUNCTION
        [](void *p) -> bool ESP_PANEL_BOARD_LCD_POST_BEGIN_FUNCTION(p),
        #else
        nullptr,
        #endif // ESP_PANEL_BOARD_LCD_POST_BEGIN_FUNCTION
        #ifdef ESP_PANEL_BOARD_BACKLIGHT_PRE_BEGIN_FUNCTION
        [](void *p) -> bool ESP_PANEL_BOARD_BACKLIGHT_PRE_BEGIN_FUNCTION(p),
        #else
        nullptr,
        #endif // ESP_PANEL_BOARD_BACKLIGHT_PRE_BEGIN_FUNCTION
        #ifdef ESP_PANEL_BOARD_BACKLIGHT_POST_BEGIN_FUNCTION
        [](void *p) -> bool ESP_PANEL_BOARD_BACKLIGHT_POST_BEGIN_FUNCTION(p),
        #else
        nullptr,
        #endif // ESP_PANEL_BOARD_BACKLIGHT_POST_BEGIN_FUNCTION
    },
};
// *INDENT-ON*
