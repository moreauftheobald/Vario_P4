/**
 * @file wifi_helper.h
 * @brief Helper WiFi simplifié (copie conforme de l'exemple qui fonctionne)
 */

#ifndef WIFI_HELPER_H
#define WIFI_HELPER_H

#include <Arduino.h>
#include "src/config/config.h"
#include <WiFi.h>
#include "src/system/logger.h"


// =============================================================================
// ÉTAT
// =============================================================================

static bool wifi_connected = false;

// =============================================================================
// FONCTIONS
// =============================================================================

/**
 * @brief Se connecte au WiFi (blocking)
 */
bool wifi_connect() {
  LOG_I(LOG_WIFI, "Connecting to: %s", wifi_ssid);
  
  WiFi.begin(wifi_ssid, wifi_password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  
  wifi_connected = true;
  
  LOG_I(LOG_WIFI, "Connected!");
  LOG_I(LOG_WIFI, "  IP: %s", WiFi.localIP().toString().c_str());
  LOG_I(LOG_WIFI, "  RSSI: %d dBm", WiFi.RSSI());
  
  return true;
}

/**
 * @brief Vérifie si connecté
 */
bool wifi_is_connected() {
  return (WiFi.status() == WL_CONNECTED);
}

/**
 * @brief Obtient l'IP
 */
String wifi_get_local_ip() {
  return WiFi.localIP().toString();
}

/**
 * @brief Obtient le RSSI
 */
int wifi_get_rssi() {
  return WiFi.RSSI();
}

#endif  // WIFI_HELPER_H