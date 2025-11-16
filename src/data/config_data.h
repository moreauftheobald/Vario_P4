/**
 * @file config_data.h
 * @brief Structures de configuration du variometre
 * 
 * Contient toutes les structures pour stocker la configuration
 * chargee depuis config.json (SD) ou config par defaut (flash).
 * 
 * @author Theobald Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#ifndef CONFIG_DATA_H
#define CONFIG_DATA_H

#include <stdint.h>
#include <stdbool.h>

// Tailles maximales
#define CONFIG_STRING_MAX 32
#define CONFIG_LOG_LEVEL_MAX 16

/**
 * @brief Configuration du systeme de logging
 * 
 * Definit la sortie et le niveau de log pour chaque module.
 */
typedef struct {
    char output[CONFIG_LOG_LEVEL_MAX];      // "UART", "File", "Both", "None"
    char kalman[CONFIG_LOG_LEVEL_MAX];      // "None", "Error", "Warning", "Info", "Verbose"
    char i2c[CONFIG_LOG_LEVEL_MAX];
    char bmp390[CONFIG_LOG_LEVEL_MAX];
    char imu[CONFIG_LOG_LEVEL_MAX];
    char gps[CONFIG_LOG_LEVEL_MAX];
    char theme[CONFIG_LOG_LEVEL_MAX];
    char display[CONFIG_LOG_LEVEL_MAX];
    char map[CONFIG_LOG_LEVEL_MAX];
    char wifi[CONFIG_LOG_LEVEL_MAX];
    char storage[CONFIG_LOG_LEVEL_MAX];
    char flight[CONFIG_LOG_LEVEL_MAX];
    char system[CONFIG_LOG_LEVEL_MAX];
    char memory[CONFIG_LOG_LEVEL_MAX];      // Monitoring mémoire (SRAM, PSRAM, fragmentation)
} logger_config_t;

/**
 * @brief Configuration des parametres de vol
 */
typedef struct {
    float vario_damping;                    // Amortissement vario (0.0 - 1.0)
    float vario_integration_time;           // Temps integration vario (secondes)
    float qnh;                              // QNH en hPa (standard: 1013.25)
    float vario_threshold_strong;           // Seuil vario fort (m/s)
    float vario_threshold_medium;           // Seuil vario moyen (m/s)
    float vario_threshold_weak;             // Seuil vario faible (m/s)
} flight_params_config_t;

/**
 * @brief Configuration de la carte OSM
 */
typedef struct {
    uint8_t zoom_default;                   // Niveau de zoom par defaut (10-18)
    uint8_t zoom_min;                       // Zoom minimum
    uint8_t zoom_max;                       // Zoom maximum
    uint16_t tile_max_age_days;             // Age max tiles avant re-telechargement
    uint16_t cache_size_mb;                 // Taille cache en MB
} map_config_t;

/**
 * @brief Configuration WiFi
 */
typedef struct {
    char ssid[CONFIG_STRING_MAX];           // SSID du reseau WiFi
    char password[CONFIG_STRING_MAX];       // Mot de passe WiFi
    bool auto_connect;                      // Connexion auto au demarrage
} wifi_config_t;

/**
 * @brief Configuration affichage
 */
typedef struct {
    uint8_t brightness;                     // Luminosite ecran (0-100)
    uint16_t refresh_rate;                  // Taux rafraichissement (Hz)
    bool auto_brightness;                   // Luminosite automatique
} display_config_t;

/**
 * @brief Calibration IMU
 */
typedef struct {
    struct {
        float x, y, z;
    } gyro_offset;
    
    struct {
        float x, y, z;
    } accel_offset;
    
    struct {
        float x, y, z;
    } accel_scale;
    
    struct {
        uint32_t timestamp;
        float temperature;
        uint8_t quality_score;
        char location[16];
    } metadata;
} imu_calibration_config_t;

/**
 * @brief Configuration complete du variometre
 * 
 * Structure principale contenant toute la configuration
 * du variometre chargee depuis JSON.
 */
typedef struct {
    logger_config_t logger;                 // Configuration logging
    flight_params_config_t flight_params;   // Parametres de vol
    map_config_t map;                       // Configuration carte
    wifi_config_t wifi;                     // Configuration WiFi
    display_config_t display;               // Configuration affichage
    imu_calibration_config_t imu_calibration;  // Calibration IMU
    uint8_t config_source;                  // 0=hardcoded, 1=LITTLEFS, 2=SD
} variometer_config_t;

// Variable globale de configuration
extern variometer_config_t g_config;

// Sources de configuration
#define CONFIG_SOURCE_HARDCODED 0
#define CONFIG_SOURCE_LITTLEFS  1
#define CONFIG_SOURCE_SD        2

#endif // CONFIG_DATA_H