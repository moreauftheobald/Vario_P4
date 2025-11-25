/**
 * @file log_level.h
 * @brief Configuration des niveaux de log par module
 */

#ifndef LOG_LEVEL_H
#define LOG_LEVEL_H

// =============================================================================
// MODULES
// =============================================================================
enum LogModule {
  LOG_SYSTEM = 0,
  LOG_DISPLAY,
  LOG_TOUCH,
  LOG_GPS,
  LOG_BARO,
  LOG_IMU,
  LOG_BATTERY,
  LOG_SD,
  LOG_WIFI,
  LOG_MAP,
  LOG_FLIGHT,
  LOG_KALMAN,
  LOG_UI,
  LOG_MODULE_COUNT
};

// =============================================================================
// NIVEAUX DE LOG
// =============================================================================
enum LogLevel {
  LOG_NONE = 0,
  LOG_ERROR = 1,
  LOG_WARN = 2,
  LOG_INFO = 3,
  LOG_DEBUG = 4,
  LOG_VERBOSE = 5
};

// =============================================================================
// CONFIGURATION PAR MODULE
// =============================================================================
static LogLevel log_levels[LOG_MODULE_COUNT] = {
  LOG_INFO,      // LOG_SYSTEM
  LOG_INFO,      // LOG_DISPLAY
  LOG_INFO,      // LOG_TOUCH
  LOG_VERBOSE,      // LOG_GPS
  LOG_VERBOSE,     // LOG_BARO
  LOG_VERBOSE,      // LOG_IMU
  LOG_VERBOSE,      // LOG_BATTERY
  LOG_INFO,      // LOG_SD
  LOG_INFO,      // LOG_WIFI
  LOG_INFO,      // LOG_MAP
  LOG_INFO,      // LOG_FLIGHT
  LOG_INFO,      // LOG_KALMAN
  LOG_INFO       // LOG_UI
};

// Noms des modules (pour affichage)
static const char* log_module_names[LOG_MODULE_COUNT] = {
  "SYS",
  "DSP",
  "TCH",
  "GPS",
  "BAR",
  "IMU",
  "BAT",
  "SD",
  "WIFI",
  "MAP",
  "FLT",
  "KAL",
  "UI"
};

#endif  // LOG_LEVEL_H