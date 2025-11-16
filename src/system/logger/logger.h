/**
 * @file logger.h
 * @brief Système de logging configurable multi-niveaux
 * 
 * Système de logging sans #ifdef DEBUG_MODE, entièrement configurable
 * via config.json. Supporte plusieurs niveaux par module et sorties
 * multiples (Serial, fichier SD).
 * 
 * @author Franck Moreau
 * @date 2025-11-15
 * @version 1.0
 * 
 * LOG DOCUMENTATION - MODULE: LOGGER
 * [ERROR]
 *   - "Failed to open log file" : Impossible d'ouvrir le fichier de log
 *   - "SD card not available" : Carte SD non disponible pour log fichier
 * 
 * [WARNING]
 *   - "Log buffer full, message dropped" : Buffer de log plein
 * 
 * [INFO]
 *   - "Logger initialized, output: %s" : Logger initialisé avec sortie
 * 
 * [VERBOSE]
 *   - "Log entry written to file" : Entrée écrite dans fichier
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <stdarg.h>

// Niveaux de log
typedef enum {
    LOG_LEVEL_NONE = 0,    // Aucun log
    LOG_LEVEL_ERROR,       // Erreurs critiques uniquement
    LOG_LEVEL_WARNING,     // Avertissements + erreurs
    LOG_LEVEL_INFO,        // Informations + warning + erreurs
    LOG_LEVEL_VERBOSE      // Tout détailler
} log_level_t;

// Sorties de log
typedef enum {
    LOG_OUTPUT_NONE = 0,   // Désactivé
    LOG_OUTPUT_UART,       // Serial uniquement
    LOG_OUTPUT_FILE,       // Fichier SD uniquement
    LOG_OUTPUT_BOTH        // Serial + fichier
} log_output_t;

// Modules de log
typedef enum {
    LOG_MODULE_KALMAN = 0,
    LOG_MODULE_I2C,
    LOG_MODULE_BMP390,
    LOG_MODULE_IMU,
    LOG_MODULE_GPS,
    LOG_MODULE_THEME,
    LOG_MODULE_DISPLAY,
    LOG_MODULE_MAP,
    LOG_MODULE_WIFI,
    LOG_MODULE_STORAGE,
    LOG_MODULE_FLIGHT,
    LOG_MODULE_SYSTEM,
    LOG_MODULE_LOGGER,
    LOG_MODULE_MEMORY,     // Monitoring mémoire (SRAM, PSRAM, Flash, fragmentation)
    LOG_MODULE_COUNT       // Nombre total de modules
} log_module_t;

/**
 * @brief Initialise le système de logging
 * 
 * Configure les niveaux de log et la sortie selon config.json.
 * Doit être appelé après config_load().
 * 
 * @return true si succès, false si erreur
 */
bool logger_init();

/**
 * @brief Configure le niveau de log pour un module
 * 
 * @param[in] module Module concerné
 * @param[in] level Niveau de log à appliquer
 */
void logger_set_level(log_module_t module, log_level_t level);

/**
 * @brief Configure la sortie de log
 * 
 * @param[in] output Type de sortie (UART, File, Both, None)
 */
void logger_set_output(log_output_t output);

/**
 * @brief Fonction principale de logging
 * 
 * Fonction centrale utilisée par les macros LOG_E, LOG_W, LOG_I, LOG_V.
 * Formate et envoie le message selon la configuration.
 * 
 * @param[in] module Module émetteur du log
 * @param[in] level Niveau du message
 * @param[in] format Chaîne de format printf
 * @param[in] ... Arguments variables pour printf
 */
void logger_print(log_module_t module, log_level_t level, const char* format, ...);

/**
 * @brief Ferme proprement le système de logging
 * 
 * Ferme le fichier de log si ouvert, flush les buffers.
 */
void logger_close();

/**
 * @brief Convertit un nom de module en enum
 * 
 * @param[in] module_name Nom du module (ex: "Kalman")
 * @return log_module_t Module correspondant ou LOG_MODULE_SYSTEM si inconnu
 */
log_module_t logger_module_from_string(const char* module_name);

/**
 * @brief Convertit un nom de niveau en enum
 * 
 * @param[in] level_name Nom du niveau (ex: "Verbose")
 * @return log_level_t Niveau correspondant ou LOG_LEVEL_INFO si inconnu
 */
log_level_t logger_level_from_string(const char* level_name);

/**
 * @brief Convertit un nom de sortie en enum
 * 
 * @param[in] output_name Nom de la sortie (ex: "UART")
 * @return log_output_t Sortie correspondante ou LOG_OUTPUT_UART si inconnu
 */
log_output_t logger_output_from_string(const char* output_name);

// Macros de convenance pour le logging
#define LOG_E(module, ...) logger_print(module, LOG_LEVEL_ERROR, __VA_ARGS__)
#define LOG_W(module, ...) logger_print(module, LOG_LEVEL_WARNING, __VA_ARGS__)
#define LOG_I(module, ...) logger_print(module, LOG_LEVEL_INFO, __VA_ARGS__)
#define LOG_V(module, ...) logger_print(module, LOG_LEVEL_VERBOSE, __VA_ARGS__)

#endif // LOGGER_H