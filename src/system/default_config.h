/**
 * @file default_config.h
 * @brief Configuration par defaut embarquee en flash
 *
 * Contient la configuration JSON par defaut utilisee si aucun
 * fichier config.json n'est present sur la carte SD ou LittleFS.
 * Stocke en PROGMEM pour economiser la RAM.
 *
 * @author Theobald Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#ifndef DEFAULT_CONFIG_H
#define DEFAULT_CONFIG_H

#include <Arduino.h>

/**
 * @brief Configuration JSON par defaut
 *
 * Cette configuration est utilisee au premier demarrage
 * ou si la carte SD et LittleFS ne sont pas accessibles.
 */
const char DEFAULT_CONFIG_JSON[] PROGMEM = R"({
  "logger": {
    "output": "UART",
    "kalman": "Info",
    "i2c": "Warning",
    "bmp390": "Info",
    "imu": "Info",
    "gps": "Info",
    "theme": "Warning",
    "display": "Info",
    "map": "Info",
    "wifi": "Info",
    "storage": "Warning",
    "flight": "Info",
    "system": "Info",
    "memory": "Info"
  },
  "flight_params": {
    "vario_damping": 0.5,
    "vario_integration_time": 2.0,
    "qnh": 1013.25,
    "vario_threshold_strong": 3.0,
    "vario_threshold_medium": 1.5,
    "vario_threshold_weak": 0.5
  },
  "map": {
    "zoom_default": 14,
    "zoom_min": 10,
    "zoom_max": 18,
    "tile_max_age_days": 180,
    "cache_size_mb": 100
  },
  "wifi": {
      "ssid": "",
      "password": "",
      "auto_connect": false
  },
  "display": {
      "brightness": 80,
      "refresh_rate": 30,
      "auto_brightness": false
  }
})";

#endif // DEFAULT_CONFIG_H