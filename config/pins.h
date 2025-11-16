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
// I2C pour capteurs (LSM6DSO32, BMP390, GPS PA1010D)
#define I2C_SDA_PIN             8           // GPIO8 - SDA
#define I2C_SCL_PIN             9           // GPIO9 - SCL

// =============================================================================
// CARTE SD (SD_MMC)
// =============================================================================
// Mode 1-bit SD_MMC
#define SD_PIN_CLK              39          // GPIO39 - SD_CLK
#define SD_PIN_CMD              40          // GPIO40 - SD_CMD
#define SD_PIN_D0               41          // GPIO41 - SD_D0

// Mode 4-bit SD_MMC (optionnel, non utilisé actuellement)
#define SD_PIN_D1               42          // GPIO42 - SD_D1
#define SD_PIN_D2               43          // GPIO43 - SD_D2
#define SD_PIN_D3               44          // GPIO44 - SD_D3

// =============================================================================
// ÉCRAN MIPI DSI (géré par la bibliothèque ESP32_Display_Panel)
// =============================================================================
// Ces pins sont gérées automatiquement par la bibliothèque
// Documentation à titre informatif uniquement
#define DISPLAY_CS_PIN          45          // Chip Select
#define DISPLAY_BACKLIGHT_PIN   46          // Rétroéclairage

// =============================================================================
// ÉCRAN TACTILE (si présent)
// =============================================================================
#define TOUCH_SDA_PIN           47          // I2C tactile SDA
#define TOUCH_SCL_PIN           48          // I2C tactile SCL
#define TOUCH_INT_PIN           -1          // Interruption tactile (optionnel)

#endif // PINS_H