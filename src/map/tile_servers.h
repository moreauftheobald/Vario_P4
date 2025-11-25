/**
 * @file tile_servers.h
 * @brief Configuration des serveurs de tiles JPEG
 */

#ifndef TILE_SERVERS_H
#define TILE_SERVERS_H

#include <Arduino.h>

// =============================================================================
// SERVEURS DE TILES JPEG - DÉCODEUR HARDWARE ESP32-P4
// =============================================================================

// ✅ ESRI World Topo - Topographie mondiale (gratuit, sans clé)
static const char tile_name_0[] PROGMEM = "Topographie";
static const char tile_url_0[] PROGMEM = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}";

// ✅ Thunderforest Landscape - Style OpenTopoMap (clé API gratuite)
// Inscription : https://manage.thunderforest.com/users/sign_up
static const char tile_name_1[] PROGMEM = "Landscape";
static const char tile_url_1[] PROGMEM = "https://tile.thunderforest.com/landscape/{z}/{x}/{y}.jpg?apikey={THUNDERFOREST_KEY}";

// ✅ Google Hybrid - Satellite + routes + labels (gratuit, peut être limité)
// lyrs=y : hybrid (satellite + labels)
// lyrs=s : satellite seul
// lyrs=p : terrain
static const char tile_name_2[] PROGMEM = "Satellite";
static const char tile_url_2[] PROGMEM = "http://mt{SERVER}.google.com/vt/lyrs=y&x={x}&y={y}&z={z}";

// =============================================================================
// CONFIGURATION
// =============================================================================

#define TILE_SERVER_COUNT 3
#define TILE_DEFAULT_SERVER 0  // ESRI Topo par défaut

// Clés API (à configurer dans config.json ou ici)
#define THUNDERFOREST_API_KEY "42ed8b78f4764cadb55b1a42623b203f"

// Paramètres de cache
#define TILE_CACHE_DIR "/tiles"
#define TILE_MAX_AGE_DAYS 180  // 6 mois
#define TILE_SIZE_BYTES 256 * 256 * 2  // Estimation max JPEG

// Structure serveur
struct TileServer {
  const char* name;
  const char* url;
  bool requires_api_key;
};

// Table des serveurs
static const TileServer tile_servers[TILE_SERVER_COUNT] = {
  { tile_name_0, tile_url_0, false },
  { tile_name_1, tile_url_1, true },
  { tile_name_2, tile_url_2, false }
};

// =============================================================================
// FONCTIONS UTILITAIRES
// =============================================================================

/**
 * @brief Construit l'URL complète d'une tile
 * @param server_idx Index du serveur (0-2)
 * @param z Niveau de zoom
 * @param x Coordonnée X
 * @param y Coordonnée Y
 * @param output Buffer de sortie (alloué par l'appelant)
 * @param max_len Taille max du buffer
 */
void tile_build_url(uint8_t server_idx, uint8_t z, uint32_t x, uint32_t y, 
                    char* output, size_t max_len) {
  if (server_idx >= TILE_SERVER_COUNT) {
    output[0] = '\0';
    return;
  }
  
  String url = String(tile_servers[server_idx].url);
  
  // Remplacer les placeholders standards
  url.replace("{z}", String(z));
  url.replace("{x}", String(x));
  url.replace("{y}", String(y));
  
  // Google : choisir serveur aléatoire (mt0 à mt3)
  if (url.indexOf("{SERVER}") >= 0) {
    url.replace("{SERVER}", String(random(0, 4)));
  }
  
  // Clé API Thunderforest
  if (tile_servers[server_idx].requires_api_key) {
    url.replace("{THUNDERFOREST_KEY}", THUNDERFOREST_API_KEY);
  }
  
  strncpy(output, url.c_str(), max_len - 1);
  output[max_len - 1] = '\0';
}

/**
 * @brief Construit le chemin de cache d'une tile sur SD
 * @param server_idx Index du serveur
 * @param z Niveau de zoom
 * @param x Coordonnée X
 * @param y Coordonnée Y
 * @param output Buffer de sortie
 * @param max_len Taille max du buffer
 * 
 * Format : /tiles/{server}/{z}/{x}/{y}.jpg
 */
void tile_build_cache_path(uint8_t server_idx, uint8_t z, uint32_t x, uint32_t y,
                           char* output, size_t max_len) {
  snprintf(output, max_len, "%s/%d/%d/%lu/%lu.jpg", 
           TILE_CACHE_DIR, server_idx, z, x, y);
}

/**
 * @brief Obtient le nom d'un serveur
 */
const char* tile_get_server_name(uint8_t server_idx) {
  if (server_idx >= TILE_SERVER_COUNT) return "Unknown";
  return tile_servers[server_idx].name;
}

/**
 * @brief Vérifie si un serveur nécessite une clé API
 */
bool tile_requires_api_key(uint8_t server_idx) {
  if (server_idx >= TILE_SERVER_COUNT) return false;
  return tile_servers[server_idx].requires_api_key;
}

/**
 * @brief Vérifie si une clé API est configurée
 */
bool tile_has_valid_api_key(uint8_t server_idx) {
  if (!tile_requires_api_key(server_idx)) return true;
  
  // Vérifier si la clé est configurée (différente de la valeur par défaut)
  return strcmp(THUNDERFOREST_API_KEY, "YOUR_API_KEY_HERE") != 0;
}

#endif  // TILE_SERVERS_H