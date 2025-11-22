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


// =========================================================================
// CONFIGURATION TACTILE GT911
// =========================================================================
#define ESP_PANEL_BOARD_USE_TOUCH (1)
#define ESP_PANEL_BOARD_TOUCH_CONTROLLER GT911

#define ESP_PANEL_BOARD_TOUCH_BUS_TYPE (ESP_PANEL_BUS_TYPE_I2C)
#define ESP_PANEL_BOARD_TOUCH_BUS_SKIP_INIT_HOST (0)  // 0/1. Typically set to 0

#define ESP_PANEL_BOARD_TOUCH_I2C_HOST_ID (0)  // Typically set to 0
#define ESP_PANEL_BOARD_TOUCH_I2C_CLK_HZ (400 * 1000)
#define ESP_PANEL_BOARD_TOUCH_I2C_SCL_PULLUP (1)  // 0/1. Typically set to 1
#define ESP_PANEL_BOARD_TOUCH_I2C_SDA_PULLUP (1)  // 0/1. Typically set to 1
#define ESP_PANEL_BOARD_TOUCH_I2C_IO_SCL (8)
#define ESP_PANEL_BOARD_TOUCH_I2C_IO_SDA (7)
#define ESP_PANEL_BOARD_TOUCH_I2C_ADDRESS (0)
#define ESP_PANEL_BOARD_TOUCH_RST_IO (23)    // Reset pin, -1 if not used
#define ESP_PANEL_BOARD_TOUCH_RST_LEVEL (0)  // Reset active level, 0: low, 1: high
#define ESP_PANEL_BOARD_TOUCH_INT_IO (-1)    // Interrupt pin, -1 if not used
#define ESP_PANEL_BOARD_TOUCH_INT_LEVEL (0)  // Interrupt active level, 0: low, 1: high
#define ESP_PANEL_BOARD_TOUCH_WIDTH (1024)   // ← Pas 800 !
#define ESP_PANEL_BOARD_TOUCH_HEIGHT (600)   // ← Pas 480 !

#endif