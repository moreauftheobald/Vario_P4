#ifndef ESP_PANEL_BOARD_CUSTOM_CONF_H
#define ESP_PANEL_BOARD_CUSTOM_CONF_H
// =========================================================================
// CONFIGURATION GLOBALE
// =========================================================================
// Version
#define ESP_PANEL_BOARD_CUSTOM_FILE_VERSION_MAJOR 1
#define ESP_PANEL_BOARD_CUSTOM_FILE_VERSION_MINOR 1
#define ESP_PANEL_BOARD_CUSTOM_FILE_VERSION_PATCH 0

// Board
#define ESP_PANEL_BOARD_DEFAULT_USE_CUSTOM (1)
#define ESP_PANEL_BOARD_NAME "Waveshare ESP32-P4 7inch EK79007"
#define ESP_PANEL_BOARD_WIDTH (1024)
#define ESP_PANEL_BOARD_HEIGHT (600)


// =========================================================================
// CONFIGURATION ECRAN
// =========================================================================
// Macros pour construction classes
#define ESP_PANEL_BOARD_USE_LCD (1)
// LCD - GÉNÉRAUX
#define ESP_PANEL_BOARD_LCD_COLOR_BITS (16)
#define ESP_PANEL_BOARD_LCD_RST_IO (33)
#define ESP_PANEL_BOARD_LCD_GAP_X (0)
#define ESP_PANEL_BOARD_LCD_GAP_Y (0)

// LCD - TYPE BUS
#define ESP_PANEL_BOARD_LCD_BUS_TYPE (ESP_PANEL_BUS_TYPE_MIPI_DSI)
#define ESP_PANEL_BOARD_LCD_BUS_NAME DSI
#define ESP_PANEL_BOARD_LCD_CONTROLLER EK79007

// MIPI DSI/DPI - BOARD
#define ESP_PANEL_BOARD_LCD_MIPI_DSI_LANE_NUM (2)
#define ESP_PANEL_BOARD_LCD_MIPI_DSI_LANE_RATE_MBPS (1000)
#define ESP_PANEL_BOARD_LCD_MIPI_DPI_CLK_MHZ (60)
#define ESP_PANEL_BOARD_LCD_MIPI_DPI_PIXEL_BITS (16)
#define ESP_PANEL_BOARD_LCD_MIPI_DPI_HPW (10)
#define ESP_PANEL_BOARD_LCD_MIPI_DPI_HBP (160)
#define ESP_PANEL_BOARD_LCD_MIPI_DPI_HFP (160)
#define ESP_PANEL_BOARD_LCD_MIPI_DPI_VPW (4)
#define ESP_PANEL_BOARD_LCD_MIPI_DPI_VBP (23)
#define ESP_PANEL_BOARD_LCD_MIPI_DPI_VFP (12)
#define ESP_PANEL_BOARD_LCD_MIPI_PHY_LDO_ID (3)
#define ESP_PANEL_BOARD_LCD_MIPI_PHY_LDO_VOL_MV (2500)


// =========================================================================
// CONFIGURATION BACKLIGHT PWM
// =========================================================================
#define ESP_PANEL_BOARD_USE_BACKLIGHT (1)
#define ESP_PANEL_BOARD_BACKLIGHT_NAME PWM_LEDC
#define ESP_PANEL_BOARD_BACKLIGHT_TYPE (ESP_PANEL_BACKLIGHT_TYPE_PWM_LEDC)
#define ESP_PANEL_BOARD_BACKLIGHT_IO (32)
#define ESP_PANEL_BOARD_BACKLIGHT_ON_LEVEL (0)


#endif
