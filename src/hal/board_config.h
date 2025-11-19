/**
 * @file board_config.h
 * @brief Configuration Waveshare ESP32-P4 7" MIPI DSI
 * 
 * Basée sur l'exemple officiel Waveshare ESP-IDF
 * https://github.com/waveshareteam/ESP32-P4-WIFI6-Touch-LCD-7B
 * 
 * @author Franck Moreau
 * @date 2025-11-19
 * @version 1.2
 */

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <ESP_Panel_Library.h>
#include "config/pins.h"

// =============================================================================
// CONFIGURATION LCD MIPI DSI (valeurs officielles Waveshare)
// =============================================================================
#define BOARD_LCD_MIPI_DSI              1

// Résolution
#define BOARD_LCD_WIDTH                 1024
#define BOARD_LCD_HEIGHT                600

// MIPI DSI - Configuration exacte Waveshare
#define BOARD_LCD_MIPI_DSI_LANE_NUM     2           // 2 lanes
#define BOARD_LCD_MIPI_DSI_LANE_MBPS    1000        // 1000 Mbps par lane

// Pixel clock (IMPORTANT: 60 MHz selon Waveshare, pas 51 MHz)
#define BOARD_LCD_RGB_TIMING_FREQ_HZ    (60 * 1000 * 1000)  // 60 MHz

// Timing horizontal (valeurs officielles Waveshare)
#define BOARD_LCD_RGB_TIMING_HPW        10          // Hsync pulse width
#define BOARD_LCD_RGB_TIMING_HBP        160         // Hsync back porch
#define BOARD_LCD_RGB_TIMING_HFP        160         // Hsync front porch

// Timing vertical (valeurs officielles Waveshare)
#define BOARD_LCD_RGB_TIMING_VPW        4           // Vsync pulse width
#define BOARD_LCD_RGB_TIMING_VBP        23          // Vsync back porch
#define BOARD_LCD_RGB_TIMING_VFP        12          // Vsync front porch

// Flags timing
#define BOARD_LCD_RGB_HSYNC_IDLE_LOW    0
#define BOARD_LCD_RGB_VSYNC_IDLE_LOW    0
#define BOARD_LCD_RGB_DE_IDLE_HIGH      0
#define BOARD_LCD_RGB_PCLK_ACTIVE_NEG   0
#define BOARD_LCD_RGB_PCLK_IDLE_HIGH    0

// Reset LCD
#define BOARD_LCD_RST_PIN               45          // GPIO45

// =============================================================================
// CONFIGURATION BACKLIGHT
// =============================================================================
#define BOARD_LCD_BL_PIN                23          // GPIO23 (BL_CTRL)
#define BOARD_LCD_BL_ON_LEVEL           1           // High = ON
#define BOARD_LCD_BL_USE_PWM            1           // PWM pour luminosité

// =============================================================================
// CONFIGURATION TOUCH GT911
// =============================================================================
#define BOARD_TOUCH_GT911               1

// I2C0 pour le tactile
#define BOARD_TOUCH_I2C_NUM             0           // I2C_NUM_0
#define BOARD_TOUCH_I2C_SCL             8           // GPIO8
#define BOARD_TOUCH_I2C_SDA             7           // GPIO7
#define BOARD_TOUCH_I2C_CLK_HZ          400000

// Adresse I2C GT911
#define BOARD_TOUCH_I2C_ADDRESS         0x5D        // ou 0x14

// Pins contrôle
#define BOARD_TOUCH_RST_PIN             23          // GPIO23 (partagé avec BL)
#define BOARD_TOUCH_INT_PIN             -1          // Non connecté

// Résolution et orientation
#define BOARD_TOUCH_WIDTH               1024
#define BOARD_TOUCH_HEIGHT              600
#define BOARD_TOUCH_SWAP_XY             0
#define BOARD_TOUCH_MIRROR_X            0
#define BOARD_TOUCH_MIRROR_Y            0

#endif // BOARD_CONFIG_H