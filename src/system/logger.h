/**
 * @file logger.h
 * @brief Système de logging simple
 * 
 * Usage:
 *   LOG_E(LOG_DISPLAY, "Erreur: %d", code);
 *   LOG_W(LOG_GPS, "Pas de fix");
 *   LOG_I(LOG_SYSTEM, "Init OK");
 *   LOG_D(LOG_KALMAN, "Valeur: %.2f", x);
 *   LOG_V(LOG_IMU, "Raw: %d %d %d", x, y, z);
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "../config/log_level.h"

// =============================================================================
// COULEURS ANSI (optionnel, pour debug UART)
// =============================================================================
#define ANSI_RESET   "\033[0m"
#define ANSI_RED     "\033[31m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_CYAN    "\033[36m"
#define ANSI_GRAY    "\033[90m"

// =============================================================================
// MACROS DE LOG
// =============================================================================

#define LOG_E(module, format, ...) \
  do { \
    if (log_levels[module] >= LOG_ERROR) { \
      Serial.printf("%s[E][%s] " format "%s\n", \
                    ANSI_RED, log_module_names[module], ##__VA_ARGS__, ANSI_RESET); \
    } \
  } while(0)

#define LOG_W(module, format, ...) \
  do { \
    if (log_levels[module] >= LOG_WARN) { \
      Serial.printf("%s[W][%s] " format "%s\n", \
                    ANSI_YELLOW, log_module_names[module], ##__VA_ARGS__, ANSI_RESET); \
    } \
  } while(0)

#define LOG_I(module, format, ...) \
  do { \
    if (log_levels[module] >= LOG_INFO) { \
      Serial.printf("%s[I][%s] " format "%s\n", \
                    ANSI_GREEN, log_module_names[module], ##__VA_ARGS__, ANSI_RESET); \
    } \
  } while(0)

#define LOG_D(module, format, ...) \
  do { \
    if (log_levels[module] >= LOG_DEBUG) { \
      Serial.printf("%s[D][%s] " format "%s\n", \
                    ANSI_CYAN, log_module_names[module], ##__VA_ARGS__, ANSI_RESET); \
    } \
  } while(0)

#define LOG_V(module, format, ...) \
  do { \
    if (log_levels[module] >= LOG_VERBOSE) { \
      Serial.printf("%s[V][%s] " format "%s\n", \
                    ANSI_GRAY, log_module_names[module], ##__VA_ARGS__, ANSI_RESET); \
    } \
  } while(0)

#endif  // LOGGER_H