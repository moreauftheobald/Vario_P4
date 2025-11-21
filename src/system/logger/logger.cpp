/**
 * @file logger.cpp
 * @brief Implémentation du système de logging
 * 
 * @author Franck Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#include "logger.h"
#include "src/data/config_data.h"
#include "src/system/sd_manager/sd_manager.h"
#include "config/config.h"
#include <time.h>

// Configuration globale du logger
static log_level_t module_levels[LOG_MODULE_COUNT];
static log_output_t log_output = LOG_OUTPUT_UART;
static File log_file;
static bool log_file_open = false;
static SemaphoreHandle_t logger_mutex = NULL;

// Noms des modules pour affichage
static const char* MODULE_NAMES[] = {
    "KALMAN",
    "I2C",
    "BMP5",
    "IMU",
    "GPS",
    "THEME",
    "DISPLAY",
    "MAP",
    "WIFI",
    "STORAGE",
    "FLIGHT",
    "SYSTEM",
    "LOGGER",
    "MEMORY"
};

// Noms des niveaux pour affichage
static const char* LEVEL_NAMES[] = {
    "NONE",
    "ERROR",
    "WARNING",
    "INFO",
    "VERBOSE"
};


/**
 * @brief Obtient la couleur ANSI selon le niveau
 */
static const char* get_level_color(log_level_t level) {
    switch (level) {
        case LOG_LEVEL_ERROR:   return ANSI_COLOR_RED;
        case LOG_LEVEL_WARNING: return ANSI_COLOR_YELLOW;
        case LOG_LEVEL_INFO:    return ANSI_COLOR_GREEN;
        case LOG_LEVEL_VERBOSE: return ANSI_COLOR_CYAN;
        default:                return ANSI_COLOR_RESET;
    }
}

/**
 * @brief Formate un timestamp pour le log
 */
static void format_timestamp(char* buffer, size_t size) {
    unsigned long ms = millis();
    unsigned long seconds = ms / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;
    
    snprintf(buffer, size, "%02lu:%02lu:%02lu.%03lu",
             hours % 24, minutes % 60, seconds % 60, ms % 1000);
}

/**
 * @brief Initialise le système de logging
 */
bool logger_init() {
    // Créer le mutex
    if (logger_mutex == NULL) {
        logger_mutex = xSemaphoreCreateMutex();
        if (logger_mutex == NULL) {
            Serial.println("[LOGGER][ERROR] Failed to create mutex");
            return false;
        }
    }
    
    // Initialiser tous les modules à INFO par défaut
    for (int i = 0; i < LOG_MODULE_COUNT; i++) {
        module_levels[i] = LOG_LEVEL_INFO;
    }
    
    // Charger configuration depuis g_config
    extern variometer_config_t g_config;
    
    // Configurer la sortie
    log_output = logger_output_from_string(g_config.logger.output);
    
    // Configurer les niveaux par module
    logger_set_level(LOG_MODULE_KALMAN, logger_level_from_string(g_config.logger.kalman));
    logger_set_level(LOG_MODULE_I2C, logger_level_from_string(g_config.logger.i2c));
    logger_set_level(LOG_MODULE_BMP5, logger_level_from_string(g_config.logger.bmp5));
    logger_set_level(LOG_MODULE_IMU, logger_level_from_string(g_config.logger.imu));
    logger_set_level(LOG_MODULE_GPS, logger_level_from_string(g_config.logger.gps));
    logger_set_level(LOG_MODULE_THEME, logger_level_from_string(g_config.logger.theme));
    logger_set_level(LOG_MODULE_DISPLAY, logger_level_from_string(g_config.logger.display));
    logger_set_level(LOG_MODULE_MAP, logger_level_from_string(g_config.logger.map));
    logger_set_level(LOG_MODULE_WIFI, logger_level_from_string(g_config.logger.wifi));
    logger_set_level(LOG_MODULE_STORAGE, logger_level_from_string(g_config.logger.storage));
    logger_set_level(LOG_MODULE_FLIGHT, logger_level_from_string(g_config.logger.flight));
    logger_set_level(LOG_MODULE_SYSTEM, logger_level_from_string(g_config.logger.system));
    logger_set_level(LOG_MODULE_MEMORY, logger_level_from_string(g_config.logger.memory));
    
    // Ouvrir le fichier de log si nécessaire
    if (log_output == LOG_OUTPUT_FILE || log_output == LOG_OUTPUT_BOTH) {
        if (!sd_manager_is_available()) {
            Serial.println("[LOGGER][ERROR] SD card not available");
            log_output = LOG_OUTPUT_UART; // Fallback sur Serial
        } else {
            // Acquisition mutex SD pour ouvrir le fichier
            if (sd_manager_lock()) {
                log_file = sd_manager_open("/logs/vario.log", FILE_APPEND);
                sd_manager_unlock();
                
                if (!log_file) {
                    Serial.println("[LOGGER][ERROR] Failed to open log file");
                    log_output = LOG_OUTPUT_UART; // Fallback sur Serial
                } else {
                    log_file_open = true;
                }
            } else {
                Serial.println("[LOGGER][ERROR] Failed to acquire SD lock");
                log_output = LOG_OUTPUT_UART; // Fallback sur Serial
            }
        }
    }
    
    // Log de confirmation
    Serial.printf("[LOGGER][INFO] Logger initialized, output: %s\n",
                  log_output == LOG_OUTPUT_UART ? "UART" :
                  log_output == LOG_OUTPUT_FILE ? "File" :
                  log_output == LOG_OUTPUT_BOTH ? "Both" : "None");
    
    return true;
}

/**
 * @brief Configure le niveau de log pour un module
 */
void logger_set_level(log_module_t module, log_level_t level) {
    if (module >= 0 && module < LOG_MODULE_COUNT) {
        module_levels[module] = level;
    }
}

/**
 * @brief Configure la sortie de log
 */
void logger_set_output(log_output_t output) {
    log_output = output;
}

/**
 * @brief Fonction principale de logging
 */
void logger_print(log_module_t module, log_level_t level, const char* format, ...) {
    // Vérifier si ce module/niveau doit être loggé
    if (module < 0 || module >= LOG_MODULE_COUNT) return;
    if (level > module_levels[module]) return;
    if (log_output == LOG_OUTPUT_NONE) return;
    
    // Prendre le mutex
    if (logger_mutex != NULL) {
        if (xSemaphoreTake(logger_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            return; // Timeout, abandon
        }
    }
    
    // Préparer le message formaté
    char message[256];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    
    // Préparer le timestamp
    char timestamp[16];
    format_timestamp(timestamp, sizeof(timestamp));
    
    // Formatter la ligne de log
    char log_line[320];
    snprintf(log_line, sizeof(log_line), "[%s][%s][%s] %s\n",
             timestamp,
             MODULE_NAMES[module],
             LEVEL_NAMES[level],
             message);
    
    // Sortie Serial avec couleurs
    if (log_output == LOG_OUTPUT_UART || log_output == LOG_OUTPUT_BOTH) {
        Serial.printf("%s%s%s",
                     get_level_color(level),
                     log_line,
                     ANSI_COLOR_RESET);
    }
    
    // Sortie fichier
    if ((log_output == LOG_OUTPUT_FILE || log_output == LOG_OUTPUT_BOTH) && log_file_open) {
        log_file.print(log_line);
        log_file.flush(); // Assurer l'écriture immédiate
    }
    
    // Libérer le mutex
    if (logger_mutex != NULL) {
        xSemaphoreGive(logger_mutex);
    }
}

/**
 * @brief Ferme proprement le système de logging
 */
void logger_close() {
    if (log_file_open) {
        log_file.close();
        log_file_open = false;
    }
    
    if (logger_mutex != NULL) {
        vSemaphoreDelete(logger_mutex);
        logger_mutex = NULL;
    }
}

/**
 * @brief Convertit un nom de module en enum
 */
log_module_t logger_module_from_string(const char* module_name) {
    if (strcasecmp(module_name, "Kalman") == 0) return LOG_MODULE_KALMAN;
    if (strcasecmp(module_name, "I2C") == 0) return LOG_MODULE_I2C;
    if (strcasecmp(module_name, "BMP5") == 0) return LOG_MODULE_BMP5;
    if (strcasecmp(module_name, "IMU") == 0) return LOG_MODULE_IMU;
    if (strcasecmp(module_name, "GPS") == 0) return LOG_MODULE_GPS;
    if (strcasecmp(module_name, "Theme") == 0) return LOG_MODULE_THEME;
    if (strcasecmp(module_name, "Display") == 0) return LOG_MODULE_DISPLAY;
    if (strcasecmp(module_name, "Map") == 0) return LOG_MODULE_MAP;
    if (strcasecmp(module_name, "WiFi") == 0) return LOG_MODULE_WIFI;
    if (strcasecmp(module_name, "Storage") == 0) return LOG_MODULE_STORAGE;
    if (strcasecmp(module_name, "Flight") == 0) return LOG_MODULE_FLIGHT;
    if (strcasecmp(module_name, "System") == 0) return LOG_MODULE_SYSTEM;
    if (strcasecmp(module_name, "Logger") == 0) return LOG_MODULE_LOGGER;
    if (strcasecmp(module_name, "Memory") == 0) return LOG_MODULE_MEMORY;
    
    return LOG_MODULE_SYSTEM; // Par défaut
}

/**
 * @brief Convertit un nom de niveau en enum
 */
log_level_t logger_level_from_string(const char* level_name) {
    if (strcasecmp(level_name, "None") == 0) return LOG_LEVEL_NONE;
    if (strcasecmp(level_name, "Error") == 0) return LOG_LEVEL_ERROR;
    if (strcasecmp(level_name, "Warning") == 0) return LOG_LEVEL_WARNING;
    if (strcasecmp(level_name, "Info") == 0) return LOG_LEVEL_INFO;
    if (strcasecmp(level_name, "Verbose") == 0) return LOG_LEVEL_VERBOSE;
    
    return LOG_LEVEL_INFO; // Par défaut
}

/**
 * @brief Convertit un nom de sortie en enum
 */
log_output_t logger_output_from_string(const char* output_name) {
    if (strcasecmp(output_name, "None") == 0) return LOG_OUTPUT_NONE;
    if (strcasecmp(output_name, "UART") == 0) return LOG_OUTPUT_UART;
    if (strcasecmp(output_name, "File") == 0) return LOG_OUTPUT_FILE;
    if (strcasecmp(output_name, "Both") == 0) return LOG_OUTPUT_BOTH;
    
    return LOG_OUTPUT_UART; // Par défaut
}