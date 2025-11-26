/**
 * @file wifi_helper.h
 * @brief Helper WiFi pour ESP32-P4 (API Arduino)
 */

#ifndef WIFI_HELPER_H
#define WIFI_HELPER_H

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "src/config/config.h"
#include "src/system/logger.h"
#include "src/hal/sd_helper.h"

// =============================================================================
// VARIABLES GLOBALES
// =============================================================================

static bool wifi_initialized = false;
static bool wifi_connected = false;
static unsigned long last_reconnect_attempt = 0;

// Structure réseau scanné
struct WiFiNetwork {
  char ssid[33];
  int8_t rssi;
  uint8_t encryption;
  int32_t channel;
};

// =============================================================================
// INITIALISATION
// =============================================================================

/**
 * @brief Initialise le WiFi
 */
bool wifi_init() {
  LOG_I(LOG_WIFI, "Initializing WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.setSleep(false);
  
  wifi_initialized = true;
  LOG_I(LOG_WIFI, "WiFi initialized");
  return true;
}

// =============================================================================
// SCAN
// =============================================================================

/**
 * @brief Scanne les réseaux WiFi
 */
int wifi_scan(WiFiNetwork* networks, int max_networks) {
  if (!wifi_initialized) {
    LOG_E(LOG_WIFI, "WiFi not initialized");
    return 0;
  }
  
  LOG_I(LOG_WIFI, "Scanning networks...");
  
  int n = WiFi.scanNetworks();
  
  if (n == 0) {
    LOG_W(LOG_WIFI, "No networks found");
    return 0;
  }
  
  LOG_I(LOG_WIFI, "Found %d networks", n);
  
  int count = (n < max_networks) ? n : max_networks;
  
  for (int i = 0; i < count; i++) {
    strncpy(networks[i].ssid, WiFi.SSID(i).c_str(), sizeof(networks[i].ssid) - 1);
    networks[i].ssid[sizeof(networks[i].ssid) - 1] = '\0';
    networks[i].rssi = WiFi.RSSI(i);
    networks[i].encryption = WiFi.encryptionType(i);
    networks[i].channel = WiFi.channel(i);
    
    LOG_I(LOG_WIFI, "  [%d] %s (RSSI: %d dBm, Ch: %d)", 
          i, networks[i].ssid, networks[i].rssi, networks[i].channel);
  }
  
  WiFi.scanDelete();
  return count;
}

/**
 * @brief Vérifie si un SSID est disponible
 */
bool wifi_is_ssid_available(const char* ssid) {
  if (!wifi_initialized) return false;
  
  int n = WiFi.scanNetworks();
  bool found = false;
  
  for (int i = 0; i < n; i++) {
    if (strcmp(WiFi.SSID(i).c_str(), ssid) == 0) {
      found = true;
      break;
    }
  }
  
  WiFi.scanDelete();
  return found;
}

// =============================================================================
// CONNEXION
// =============================================================================

/**
 * @brief Se connecte à un réseau WiFi
 */
bool wifi_connect(const char* ssid, const char* password, uint32_t timeout_ms = WIFI_CONNECT_TIMEOUT_MS) {
  if (!wifi_initialized) {
    LOG_E(LOG_WIFI, "WiFi not initialized");
    return false;
  }
  
  LOG_I(LOG_WIFI, "Connecting to: %s", ssid);
  
  // Déconnexion si déjà connecté
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
    delay(100);
  }
  
  // Connexion
  WiFi.begin(ssid, password);
  
  // Attendre connexion
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    
    LOG_I(LOG_WIFI, "Connected!");
    LOG_I(LOG_WIFI, "  IP: %s", WiFi.localIP().toString().c_str());
    LOG_I(LOG_WIFI, "  RSSI: %d dBm", WiFi.RSSI());
    LOG_I(LOG_WIFI, "  Channel: %d", WiFi.channel());
    
    return true;
  } else {
    wifi_connected = false;
    LOG_E(LOG_WIFI, "Connection failed (timeout)");
    return false;
  }
}

/**
 * @brief Déconnecte le WiFi
 */
void wifi_disconnect() {
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
    wifi_connected = false;
    LOG_I(LOG_WIFI, "Disconnected");
  }
}

/**
 * @brief Tente une reconnexion
 */
bool wifi_reconnect() {
  if (millis() - last_reconnect_attempt < WIFI_RECONNECT_INTERVAL_MS) {
    return false;
  }
  
  last_reconnect_attempt = millis();
  LOG_I(LOG_WIFI, "Attempting reconnection...");
  return wifi_connect(wifi_ssid, wifi_password);
}

// =============================================================================
// ÉTAT
// =============================================================================

/**
 * @brief Vérifie si connecté
 */
bool wifi_is_connected() {
  bool status = (WiFi.status() == WL_CONNECTED);
  wifi_connected = status;
  return status;
}

/**
 * @brief Obtient la qualité du signal (0-100%)
 */
int wifi_get_signal_quality() {
  if (!wifi_is_connected()) return 0;
  
  int rssi = WiFi.RSSI();
  int quality = 2 * (rssi + 100);
  
  if (quality > 100) quality = 100;
  if (quality < 0) quality = 0;
  
  return quality;
}

/**
 * @brief Obtient le RSSI
 */
int wifi_get_rssi() {
  if (!wifi_is_connected()) return -100;
  return WiFi.RSSI();
}

/**
 * @brief Obtient l'IP locale
 */
String wifi_get_local_ip() {
  if (!wifi_is_connected()) return "0.0.0.0";
  return WiFi.localIP().toString();
}

/**
 * @brief Obtient le SSID actuel
 */
String wifi_get_ssid() {
  if (!wifi_is_connected()) return "";
  return WiFi.SSID();
}

// =============================================================================
// TÉLÉCHARGEMENT
// =============================================================================

/**
 * @brief Télécharge un fichier via HTTP/HTTPS
 */
int wifi_download(const char* url, uint8_t* buffer, size_t max_size, uint32_t timeout_ms = 10000) {
  if (!wifi_is_connected() || !buffer) {
    LOG_E(LOG_WIFI, "WiFi not connected or invalid buffer");
    return -1;
  }
  
  HTTPClient http;
  http.setTimeout(timeout_ms);
  
  LOG_D(LOG_WIFI, "Downloading: %s", url);
  
  if (!http.begin(url)) {
    LOG_E(LOG_WIFI, "Failed to begin HTTP");
    return -1;
  }
  
  int httpCode = http.GET();
  
  if (httpCode != HTTP_CODE_OK) {
    LOG_E(LOG_WIFI, "HTTP error: %d", httpCode);
    http.end();
    return -1;
  }
  
  int content_length = http.getSize();
  
  if (content_length > (int)max_size) {
    LOG_E(LOG_WIFI, "Content too large: %d > %d", content_length, max_size);
    http.end();
    return -1;
  }
  
  WiFiClient* stream = http.getStreamPtr();
  size_t downloaded = 0;
  
  while (http.connected() && (downloaded < max_size)) {
    size_t available = stream->available();
    if (available > 0) {
      size_t to_read = (available < (max_size - downloaded)) ? available : (max_size - downloaded);
      downloaded += stream->readBytes(buffer + downloaded, to_read);
    }
    delay(1);
  }
  
  http.end();
  
  LOG_D(LOG_WIFI, "Downloaded %zu bytes", downloaded);
  return downloaded;
}

/**
 * @brief Télécharge vers fichier SD
 */
bool wifi_download_to_file(const char* url, const char* dest_path) {
  if (!wifi_is_connected()) {
    LOG_E(LOG_WIFI, "WiFi not connected");
    return false;
  }
  
  LOG_I(LOG_WIFI, "Downloading to: %s", dest_path);
  
  HTTPClient http;
  http.setTimeout(10000);
  
  if (!http.begin(url)) {
    LOG_E(LOG_WIFI, "Failed to begin HTTP");
    return false;
  }
  
  int httpCode = http.GET();
  
  if (httpCode != HTTP_CODE_OK) {
    LOG_E(LOG_WIFI, "HTTP error: %d", httpCode);
    http.end();
    return false;
  }
  
  File file = SD_MMC.open(dest_path, FILE_WRITE);
  if (!file) {
    LOG_E(LOG_WIFI, "Failed to open file: %s", dest_path);
    http.end();
    return false;
  }
  
  WiFiClient* stream = http.getStreamPtr();
  uint8_t buffer[512];
  size_t total = 0;
  
  while (http.connected()) {
    size_t available = stream->available();
    if (available > 0) {
      size_t to_read = (available < sizeof(buffer)) ? available : sizeof(buffer);
      size_t read = stream->readBytes(buffer, to_read);
      file.write(buffer, read);
      total += read;
    }
    delay(1);
  }
  
  file.close();
  http.end();
  
  LOG_I(LOG_WIFI, "Downloaded %zu bytes", total);
  return true;
}

// =============================================================================
// UTILITAIRES
// =============================================================================

/**
 * @brief Affiche les infos WiFi
 */
void wifi_print_info() {
  if (!wifi_is_connected()) {
    LOG_I(LOG_WIFI, "WiFi: Not connected");
    return;
  }
  
  LOG_I(LOG_WIFI, "╔════════════════════════════════════════╗");
  LOG_I(LOG_WIFI, "║          WiFi Information             ║");
  LOG_I(LOG_WIFI, "╠════════════════════════════════════════╣");
  LOG_I(LOG_WIFI, "║ SSID:    %-29s ║", WiFi.SSID().c_str());
  LOG_I(LOG_WIFI, "║ IP:      %-29s ║", WiFi.localIP().toString().c_str());
  LOG_I(LOG_WIFI, "║ Gateway: %-29s ║", WiFi.gatewayIP().toString().c_str());
  LOG_I(LOG_WIFI, "║ DNS:     %-29s ║", WiFi.dnsIP().toString().c_str());
  LOG_I(LOG_WIFI, "║ RSSI:    %d dBm (%d%%)                  ║", 
        WiFi.RSSI(), wifi_get_signal_quality());
  LOG_I(LOG_WIFI, "║ Channel: %-29d ║", WiFi.channel());
  LOG_I(LOG_WIFI, "╚════════════════════════════════════════╝");
}

/**
 * @brief Convertit encryption type en string
 */
const char* wifi_encryption_type_str(uint8_t type) {
  switch (type) {
    case WIFI_AUTH_OPEN: return "Open";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-E";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    default: return "Unknown";
  }
}

#endif  // WIFI_HELPER_H