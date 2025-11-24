/**
 * @file pins.h
 * @brief Définition des GPIO ESP32-P4
 */

#ifndef PINS_H
#define PINS_H

// =============================================================================
// I2C CAPTEURS (Wire1)
// =============================================================================
#define I2C_SENSORS_SDA 50
#define I2C_SENSORS_SCL 49

// =============================================================================
// SD CARD (SD_MMC 4-bit)
// =============================================================================
#define SD_CLK 43   // GPIO43 - SD_CLK
#define SD_CMD 44   // GPIO44 - SD_CMD
#define SD_D0  39   // GPIO39 - SD_D0
#define SD_D1  40   // GPIO40 - SD_D1
#define SD_D2  41   // GPIO41 - SD_D2
#define SD_D3  42   // GPIO42 - SD_D3

// =============================================================================
// TOUCH GT911 (Wire - géré par ESP32_Display_Panel)
// =============================================================================
#define TOUCH_RST 23  // Reset GPIO (géré manuellement)
#define TOUCH_INT -1  // Interrupt GPIO (nécessaire pour la séquence de reset)

#endif  // PINS_Hc