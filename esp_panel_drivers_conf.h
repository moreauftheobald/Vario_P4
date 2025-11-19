/**
 * @file esp_panel_drivers_conf.h
 * @brief Configuration des drivers ESP_Panel pour Waveshare ESP32-P4 7"
 * 
 * Placez ce fichier à la racine de votre sketch Arduino
 */

#pragma once

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////// Bus Configurations //////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Activer SEULEMENT le bus MIPI DSI (économise mémoire)
 */
#define ESP_PANEL_DRIVERS_BUS_USE_ALL                   (0)
#define ESP_PANEL_DRIVERS_BUS_USE_SPI                   (0)
#define ESP_PANEL_DRIVERS_BUS_USE_QSPI                  (0)
#define ESP_PANEL_DRIVERS_BUS_USE_RGB                   (0)
#define ESP_PANEL_DRIVERS_BUS_USE_I2C                   (1)
#define ESP_PANEL_DRIVERS_BUS_USE_MIPI_DSI              (1)     // ✅ ACTIVÉ

#define ESP_PANEL_DRIVERS_BUS_COMPILE_UNUSED_DRIVERS    (0)     // Ne pas compiler les autres

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// LCD Configurations ///////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Activer SEULEMENT le driver EK79007
 */
#define ESP_PANEL_DRIVERS_LCD_USE_ALL                   (0)
#define ESP_PANEL_DRIVERS_LCD_USE_AXS15231B             (0)
#define ESP_PANEL_DRIVERS_LCD_USE_EK9716B               (0)
#define ESP_PANEL_DRIVERS_LCD_USE_EK79007               (1)     // ✅ ACTIVÉ
#define ESP_PANEL_DRIVERS_LCD_USE_GC9A01                (0)
#define ESP_PANEL_DRIVERS_LCD_USE_GC9B71                (0)
#define ESP_PANEL_DRIVERS_LCD_USE_GC9503                (0)
#define ESP_PANEL_DRIVERS_LCD_USE_HX8399                (0)
#define ESP_PANEL_DRIVERS_LCD_USE_ILI9341               (0)
#define ESP_PANEL_DRIVERS_LCD_USE_ILI9881C              (0)
#define ESP_PANEL_DRIVERS_LCD_USE_JD9165                (0)
#define ESP_PANEL_DRIVERS_LCD_USE_JD9365                (0)
#define ESP_PANEL_DRIVERS_LCD_USE_NV3022B               (0)
#define ESP_PANEL_DRIVERS_LCD_USE_SH8601                (0)
#define ESP_PANEL_DRIVERS_LCD_USE_SPD2010               (0)
#define ESP_PANEL_DRIVERS_LCD_USE_ST7262                (0)
#define ESP_PANEL_DRIVERS_LCD_USE_ST7701                (0)
#define ESP_PANEL_DRIVERS_LCD_USE_ST7703                (0)
#define ESP_PANEL_DRIVERS_LCD_USE_ST7789                (0)
#define ESP_PANEL_DRIVERS_LCD_USE_ST7796                (0)
#define ESP_PANEL_DRIVERS_LCD_USE_ST77903               (0)
#define ESP_PANEL_DRIVERS_LCD_USE_ST77916               (0)
#define ESP_PANEL_DRIVERS_LCD_USE_ST77922               (0)

#define ESP_PANEL_DRIVERS_LCD_COMPILE_UNUSED_DRIVERS    (0)     // Ne pas compiler les autres

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////// Touch Configurations /////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Désactiver le tactile (conflit I2C)
 */
#define ESP_PANEL_DRIVERS_TOUCH_MAX_POINTS              (10)
#define ESP_PANEL_DRIVERS_TOUCH_MAX_BUTTONS             (5)

#define ESP_PANEL_DRIVERS_TOUCH_USE_ALL                 (0)
#define ESP_PANEL_DRIVERS_TOUCH_USE_AXS15231B           (0)
#define ESP_PANEL_DRIVERS_TOUCH_USE_CHSC6540            (0)
#define ESP_PANEL_DRIVERS_TOUCH_USE_CST816S             (0)
#define ESP_PANEL_DRIVERS_TOUCH_USE_CST820              (0)
#define ESP_PANEL_DRIVERS_TOUCH_USE_FT5x06              (0)
#define ESP_PANEL_DRIVERS_TOUCH_USE_GT911               (1)
#define ESP_PANEL_DRIVERS_TOUCH_USE_GT1151              (0)
#define ESP_PANEL_DRIVERS_TOUCH_USE_SPD2010             (0)
#define ESP_PANEL_DRIVERS_TOUCH_USE_ST1633              (0)
#define ESP_PANEL_DRIVERS_TOUCH_USE_ST7123              (0)
#define ESP_PANEL_DRIVERS_TOUCH_USE_STMPE610            (0)
#define ESP_PANEL_DRIVERS_TOUCH_USE_TT21100             (0)
#define ESP_PANEL_DRIVERS_TOUCH_USE_XPT2046             (0)

#define ESP_PANEL_DRIVERS_TOUCH_COMPILE_UNUSED_DRIVERS  (0)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// IO Expander Configurations //////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Pas d'IO Expander
 */
#define ESP_PANEL_DRIVERS_EXPANDER_USE_ALL              (0)
#define ESP_PANEL_DRIVERS_EXPANDER_USE_CH422G           (0)
#define ESP_PANEL_DRIVERS_EXPANDER_USE_HT8574           (0)
#define ESP_PANEL_DRIVERS_EXPANDER_USE_TCA95XX_8BIT     (0)
#define ESP_PANEL_DRIVERS_EXPANDER_USE_TCA95XX_16BIT    (0)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////// Backlight Configurations ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Activer backlight PWM
 */
#define ESP_PANEL_DRIVERS_BACKLIGHT_USE_ALL                     (0)
#define ESP_PANEL_DRIVERS_BACKLIGHT_USE_SWITCH_GPIO             (0)
#define ESP_PANEL_DRIVERS_BACKLIGHT_USE_SWITCH_EXPANDER         (0)
#define ESP_PANEL_DRIVERS_BACKLIGHT_USE_PWM_LEDC                (1)     // ✅ PWM pour backlight
#define ESP_PANEL_DRIVERS_BACKLIGHT_USE_CUSTOM                  (0)

#define ESP_PANEL_DRIVERS_BACKLIGHT_COMPILE_UNUSED_DRIVERS      (0)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////// File Version ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Version du fichier de config
 */
#define ESP_PANEL_DRIVERS_CONF_FILE_VERSION_MAJOR 1
#define ESP_PANEL_DRIVERS_CONF_FILE_VERSION_MINOR 1
#define ESP_PANEL_DRIVERS_CONF_FILE_VERSION_PATCH 0