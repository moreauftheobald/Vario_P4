/**
 * @file pins.h
 * @brief Définition des broches GPIO pour ESP32-P4
 * 
 * Toutes les affectations de pins sont centralisées ici.
 * Compatible avec la carte Waveshare ESP32-P4 7" MIPI DSI.
 * 
 * @author Franck Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#ifndef PINS_H
#define PINS_H

// =============================================================================
// COMMUNICATION I2C
// =============================================================================
// I2C pour capteurs (LSM6DSO32, BMP5, GPS PA1010D)
#define I2C_SDA_PIN             50          // GPIO8 - SDA
#define I2C_SCL_PIN             49          // GPIO9 - SCL

// =============================================================================
// CARTE SD (SD_MMC)
// =============================================================================
// Mode 1-bit SD_MMC
#define SD_PIN_CLK              43          // GPIO39 - SD_CLK
#define SD_PIN_CMD              44          // GPIO40 - SD_CMD
#define SD_PIN_D0               39          // GPIO41 - SD_D0

// Mode 4-bit SD_MMC (optionnel, non utilisé actuellement)
#define SD_PIN_D1               40          // GPIO42 - SD_D1
#define SD_PIN_D2               41          // GPIO43 - SD_D2
#define SD_PIN_D3               42          // GPIO44 - SD_D3

// =============================================================================
// ÉCRAN MIPI DSI (géré par la bibliothèque ESP32_Display_Panel)
// =============================================================================
// Ces pins sont gérées automatiquement par la bibliothèque
// Documentation à titre informatif uniquement
#define DISPLAY_CS_PIN          45          // Chip Select
#define DISPLAY_BACKLIGHT_PIN   32          // Rétroéclairage

// =============================================================================
// ÉCRAN TACTILE (I2C0)
// =============================================================================
#define TOUCH_I2C_SDA_PIN        7          // GPIO47 - Touch SDA
#define TOUCH_I2C_SCL_PIN        8          // GPIO48 - Touch SCL
#define TOUCH_I2C_INT_PIN       -1          // Pas d'interruption (optionnel)

#endif // PINS_H