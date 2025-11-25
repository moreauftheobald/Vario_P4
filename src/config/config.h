/**
 * @file config.h
 * @brief Configuration globale
 */

#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// PROJET
// =============================================================================
#define PROJECT_NAME "Variomètre ESP32-P4"
#define PROJECT_VERSION "2.0.0"

// =============================================================================
// SYSTEM
// =============================================================================
#define SENSORS_TASK_STACK_SIZE 4096

// =============================================================================
// DISPLAY
// =============================================================================
#define DISPLAY_WIDTH 1024
#define DISPLAY_HEIGHT 600
//#define DISPLAY_BUFFER_HEIGHT 120  // Hauteur buffer LVGL (lignes)
#define DISPLAY_USE_FULL_BUFFERS true

// =============================================================================
// I2C CAPTEURS
// =============================================================================
#define I2C_SENSORS_FREQ 400000  // 100kHz

// =============================================================================
// SD CARD
// =============================================================================
#define SD_MMC_MODE_4BIT true   // Mode 4-bit (false = 1-bit)

// =============================================================================
// IMU
// =============================================================================
#define USE_BNO085  // Décommenter pour BNO085, commenter pour LSM6DSO32


// =============================================================================
// WIFI
// =============================================================================
#define WIFI_CONNECT_TIMEOUT_MS 10000
#define WIFI_SCAN_MAX_NETWORKS 20
#define WIFI_RECONNECT_INTERVAL_MS 30000

// Credentials par défaut
#define WIFI_DEFAULT_SSID "NAWAK"
#define WIFI_DEFAULT_PASSWORD "123456789"
#endif  // CONFIG_H