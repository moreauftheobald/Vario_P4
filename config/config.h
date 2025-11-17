/**
 * @file config.h
 * @brief Configuration globale du variomètre
 * 
 * Toutes les constantes du projet sont centralisées ici.
 * Aucun #define ne doit être fait ailleurs dans le code.
 * 
 * @author Franck Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// INFORMATIONS PROJET
// =============================================================================
#define PROJECT_NAME            "Variometer ESP32-P4"
#define PROJECT_VERSION         "1.0.0"
#define PROJECT_AUTHOR          "Theobald Moreau"
#define HARDWARE_VERSION        "1.0"
#define COMPILATION_DATE        __DATE__
#define COMPILATION_TIME        __TIME__

// =============================================================================
// CONFIGURATION I2C
// =============================================================================
#define I2C_PORT                0           // Port I2C (0 ou 1)
#define I2C_FREQUENCY           400000      // 400 kHz (standard rapide)
#define I2C_TIMEOUT_MS          100         // Timeout transactions I2C

// Note: Les pins SDA/SCL sont dans pins.h

// =============================================================================
// ADRESSES I2C CAPTEURS
// =============================================================================
#define LSM6DSO32_I2C_ADDR      0x6A        // IMU (ou 0x6B selon SDO)
#define BMP390_I2C_ADDR         0x77        // Baromètre (ou 0x76 selon SDO)
#define GPS_I2C_ADDR            0x10        // GPS PA1010D (adresse I2C)

// =============================================================================
// CONFIGURATION CAPTEURS - LSM6DSO32
// =============================================================================
// Plages de mesure
#define LSM6DSO32_ACCEL_RANGE   LSM6DSO32_ACCEL_RANGE_8_G      // ±8G (parapente)
#define LSM6DSO32_GYRO_RANGE    LSM6DS_GYRO_RANGE_125_DPS      // ±125°/s (précision max)

// Fréquences d'échantillonnage
#define LSM6DSO32_ACCEL_ODR     LSM6DS_RATE_104_HZ              // 208 Hz
#define LSM6DSO32_GYRO_ODR      LSM6DS_RATE_104_HZ              // 208 Hz

// Filtres
#define LSM6DSO32_ACCEL_FILTER  LSM6DS_ACCEL_LPF1_ODR_DIV_4    // Low-pass filtre
#define LSM6DSO32_GYRO_FILTER   LSM6DS_GYRO_LPF1_ODR_DIV_4     // Low-pass filtre

// =============================================================================
// CONFIGURATION CAPTEURS - BMP390
// =============================================================================
// Oversampling
#define BMP390_TEMP_OVERSAMPLE  BMP3_OVERSAMPLING_8X            // Température 8x
#define BMP390_PRESS_OVERSAMPLE BMP3_OVERSAMPLING_32X           // Pression 32x (max précision)

// Filtre IIR
#define BMP390_IIR_FILTER       BMP3_IIR_FILTER_COEFF_3         // Filtre IIR coeff 3

// Fréquence
#define BMP390_OUTPUT_DATA_RATE BMP3_ODR_50_HZ                  // 50 Hz

// =============================================================================
// CONFIGURATION CAPTEURS - GPS PA1010D
// =============================================================================
#define GPS_UPDATE_RATE_HZ      2           // 1 Hz (défaut PA1010D)
#define GPS_I2C_BUFFER_SIZE     256         // Buffer réception

// =============================================================================
// FRÉQUENCES SYSTÈME
// =============================================================================
// Tâche sensors (lecture capteurs)
#define TASK_SENSORS_FREQ_HZ    100         // Fréquence principale (LSM6DSO32)
#define TASK_SENSORS_PERIOD_MS  (1000 / TASK_SENSORS_FREQ_HZ)

// Diviseurs de fréquence (sous-échantillonnage)
#define FREQ_BMP390_DIVIDER     2           // BMP390 à 100/2 = 50 Hz
#define FREQ_GPS_DIVIDER        50          // GPS à 100/50 = 2 Hz
#define FREQ_KALMAN_DIVIDER     1           // Kalman à 100/1 = 100 Hz
#define FREQ_FUSION_DIVIDER     1           // Madgwick à 100/1 = 100 Hz

// Fréquences résultantes (pour info)
#define FREQ_IMU_HZ             200
#define FREQ_BARO_HZ            50
#define FREQ_GPS_HZ             2
#define FREQ_KALMAN_HZ          100
#define FREQ_FUSION_HZ          200

// Autres tâches
#define TASK_DISPLAY_FREQ_HZ    30          // Rafraîchissement écran
#define TASK_STORAGE_FREQ_HZ    1           // Enregistrement SD (1 Hz)

// =============================================================================
// CONFIGURATION TÂCHES FREERTOS
// =============================================================================
// Tailles de stack (en mots de 32 bits)
#define TASK_SENSORS_STACK_SIZE     8192    // Tâche sensors (critique)
#define TASK_DISPLAY_STACK_SIZE     8192    // LVGL (gourmand)
#define TASK_STORAGE_STACK_SIZE     4096    // Enregistrement SD

// Priorités (0 = plus basse, 24 = plus haute sur ESP32)
#define TASK_SENSORS_PRIORITY       5       // Très haute (temps réel capteurs)
#define TASK_DISPLAY_PRIORITY       3       // Moyenne (UI)
#define TASK_STORAGE_PRIORITY       1       // Très basse (background)

// Affectation cores (ESP32-P4 dual-core)
#define TASK_SENSORS_CORE           1       // Core 1 (dédié capteurs)
#define TASK_DISPLAY_CORE           0       // Core 0 (avec LVGL)
#define TASK_STORAGE_CORE           0       // Core 0

// =============================================================================
// CALIBRATION IMU
// =============================================================================
// Seuils de détection immobilité
#define IMU_STATIONARY_GYRO_THRESHOLD   0.1f    // rad/s (peu de rotation)
#define IMU_STATIONARY_ACCEL_THRESHOLD  0.3f    // m/s² (peu d'accélération)
#define IMU_STATIONARY_SAMPLES          50      // Nb échantillons pour test
#define IMU_STATIONARY_DURATION_MS      250     // Durée test (50 * 5ms)

// Seuils qualité calibration
#define IMU_CAL_GRAVITY_TOLERANCE_EXCELLENT  0.1f   // ±0.1 m/s²
#define IMU_CAL_GRAVITY_TOLERANCE_GOOD       0.3f   // ±0.3 m/s²
#define IMU_CAL_GYRO_DRIFT_EXCELLENT         0.05f  // ±0.05 rad/s
#define IMU_CAL_GYRO_DRIFT_GOOD              0.15f  // ±0.15 rad/s
#define IMU_CAL_TEMP_DIFF_WARNING            10.0f  // ±10°C
#define IMU_CAL_AGE_WARNING_DAYS             30     // 30 jours
#define IMU_CAL_AGE_ERROR_DAYS               90     // 90 jours

// Scores calibration
#define IMU_CAL_SCORE_EXCELLENT     90      // Score ≥90 = excellent
#define IMU_CAL_SCORE_GOOD          70      // Score ≥70 = bon
#define IMU_CAL_SCORE_ACCEPTABLE    60      // Score ≥60 = acceptable
#define IMU_CAL_SCORE_BAD           50      // Score <50 = mauvais

// Nombres d'échantillons pour calibration
#define IMU_CAL_SAMPLES_QUICK       100     // Calibration rapide (offset gyro)
#define IMU_CAL_SAMPLES_FULL        200     // Calibration complète

// =============================================================================
// FILTRE DE KALMAN
// =============================================================================
// Paramètres fusion altitude/vario
#define KALMAN_PROCESS_NOISE_ALT    0.01f   // Bruit processus altitude
#define KALMAN_PROCESS_NOISE_VARIO  0.1f    // Bruit processus vario
#define KALMAN_MEASURE_NOISE_BARO   0.5f    // Bruit mesure baromètre (m)
#define KALMAN_MEASURE_NOISE_IMU    2.0f    // Bruit mesure IMU (m/s²)

// =============================================================================
// FUSION MADGWICK
// =============================================================================
#define MADGWICK_BETA               0.1f    // Gain fusion (0.033-0.1 typique)
#define MADGWICK_SAMPLE_FREQ        FREQ_FUSION_HZ

// =============================================================================
// CONSTANTES PHYSIQUES
// =============================================================================
#define GRAVITY_STANDARD            9.80665f    // m/s² (gravité standard)
#define QNH_STANDARD                1013.25f    // hPa (pression niveau mer standard)
#define TEMP_LAPSE_RATE             -0.0065f    // K/m (gradient température atmosphère)
#define EARTH_RADIUS                6371000.0f  // m (rayon terre moyen)

// =============================================================================
// SEUILS VARIO
// =============================================================================
#define VARIO_THRESHOLD_STRONG      3.0f    // m/s (montée forte)
#define VARIO_THRESHOLD_MEDIUM      1.5f    // m/s (montée moyenne)
#define VARIO_THRESHOLD_WEAK        0.5f    // m/s (montée faible)
#define VARIO_THRESHOLD_SINK        -2.0f   // m/s (descente notable)

// =============================================================================
// MÉMOIRE
// =============================================================================
#define MEMORY_CRITICAL_THRESHOLD   20480   // 20 KB (seuil critique)
#define MEMORY_LOW_THRESHOLD        51200   // 50 KB (seuil bas)
#define MEMORY_FRAG_THRESHOLD       0.5f    // 50% (seuil fragmentation)

// =============================================================================
// TIMEOUTS ET DÉLAIS
// =============================================================================
#define TIMEOUT_SENSOR_INIT_MS      5000    // Timeout init capteurs
#define TIMEOUT_GPS_FIX_MS          60000   // 60s pour premier fix GPS
#define TIMEOUT_SD_OPERATION_MS     1000    // 1s opérations SD

#define DELAY_STARTUP_MS            1000    // Délai démarrage initial
#define DELAY_CALIBRATION_MSG_MS    2000    // Affichage message calibration

// Constantes détection défaillance
#define SENSOR_ERROR_THRESHOLD      10    // 10 erreurs consécutives = défaillant
#define SENSOR_TIMEOUT_MS           5000  // 5s sans lecture = défaillant

// =============================================================================
// AFFICHAGE
// =============================================================================
#define DISPLAY_BRIGHTNESS_DEFAULT  80      // Luminosité par défaut (0-100)
#define DISPLAY_REFRESH_RATE_HZ     30      // Taux rafraîchissement

// =============================================================================
// CARTE SD
// =============================================================================
#define SD_MMC_MODE_1BIT            true    // Mode 1-bit (ou false pour 4-bit)
#define SD_MAX_FILES_OPEN           5       // Nb max fichiers ouverts simultanément

// =============================================================================
// LOGGING
// =============================================================================
#define LOGGER_BUFFER_SIZE          512     // Taille buffer log
#define LOGGER_FILE_MAX_SIZE_MB     10      // Taille max fichier log (MB)
#define LOGGER_USE_COLORS           0       // Couleurs ANSI (0=off, 1=on)
// Activer/désactiver les couleurs ANSI (désactiver pour Arduino IDE)
#define LOGGER_USE_COLORS 0  // 0 = désactivé, 1 = activé

// Couleurs ANSI pour Serial (optionnel)
#if LOGGER_USE_COLORS
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"
#else
#define ANSI_COLOR_RED     ""
#define ANSI_COLOR_YELLOW  ""
#define ANSI_COLOR_GREEN   ""
#define ANSI_COLOR_CYAN    ""
#define ANSI_COLOR_RESET   ""
#endif

// =============================================================================
// WATCHDOG
// =============================================================================
#define WATCHDOG_TIMEOUT_MS         10000   // 10s watchdog
#define WATCHDOG_ENABLE             true    // Activer watchdog

// =============================================================================
// DATA CONFIG.H/CPP
// =============================================================================
// Tailles maximales
#define CONFIG_STRING_MAX 32
#define CONFIG_LOG_LEVEL_MAX 16
// Sources de configuration
#define CONFIG_SOURCE_HARDCODED 0
#define CONFIG_SOURCE_LITTLEFS  1
#define CONFIG_SOURCE_SD        2

// =============================================================================
// CONFIG_LOADER.H/CPP
// =============================================================================
// Chemins des fichiers de configuration
#define CONFIG_PATH_SD "/config/config.json"
#define CONFIG_PATH_LITTLEFS "/config.json"

// =============================================================================
// TASK_FLIGHT.H/CPP
// =============================================================================
#define VARIO_HISTORY_SIZE 30

// =============================================================================
// DEBUG / DÉVELOPPEMENT
// =============================================================================
#define DEBUG_PRINT_SENSOR_RAW      1       // Afficher données brutes capteurs
#define DEBUG_PRINT_FUSION_OUTPUT   1       // Afficher sortie fusion
#define DEBUG_PRINT_KALMAN_STATE    1       // Afficher état Kalman

#endif // CONFIG_H