/**
 * @file flight_data.h
 * @brief Structure centralisée des données de vol
 * 
 * Partagée entre:
 * - Tâche capteurs (écriture)
 * - Tâche UI/Display (lecture)
 * 
 * Protégée par mutex pour thread-safety
 */

#ifndef FLIGHT_DATA_H
#define FLIGHT_DATA_H

#include <Arduino.h>

// =============================================================================
// STRUCTURE DES DONNÉES DE VOL
// =============================================================================

struct FlightData {
  // === ALTITUDES ===
  float altitude_gps;          // Altitude GPS (m)
  float altitude_qne;          // Altitude QNE 1013.25 hPa (m)
  float altitude_qnh;          // Altitude QNH locale (m)
  float altitude_qfe;          // Altitude QFE / AGL (m)
  float altitude_takeoff;      // Altitude décollage (m)
  float altitude_max;          // Altitude max du vol (m)
  float altitude_min;          // Altitude min du vol (m)
  
  // === VARIO ===
  float vario;                 // Vario instantané (m/s)
  float vario_integrated;      // Vario intégré sur 3s (m/s)
  float vario_averaged;        // Vario moyenné (m/s)
  float vario_max;             // Vario max (m/s)
  float vario_min;             // Vario min (m/s)
  
  // === VITESSES ===
  float speed_gps;             // Vitesse GPS (km/h)
  float speed_air;             // Vitesse air estimée (km/h)
  float speed_vertical;        // Vitesse verticale (m/s)
  float speed_max;             // Vitesse max (km/h)
  
  // === FINESSE ===
  float glide_ratio;           // Finesse instantanée
  float glide_ratio_avg;       // Finesse moyenne sur 30s
  float glide_ratio_best;      // Meilleure finesse du vol
  
  // === DISTANCES ===
  float distance_takeoff;      // Distance depuis décollage (km)
  float distance_linear;       // Distance linéaire (km)
  float distance_total;        // Distance cumulée (km)
  
  // === CAP ===
  float heading;               // Cap GPS (degrés)
  float heading_takeoff;       // Cap vers décollage (degrés)
  
  // === THERMIQUE ===
  float thermal_gain;          // Gain dans thermique actuel (m)
  float thermal_avg_vario;     // Vario moyen dans thermique (m/s)
  float thermal_time;          // Temps dans thermique (s)
  bool in_thermal;             // Dans un thermique?
  
  // === VENT ===
  float wind_speed;            // Vitesse du vent (km/h)
  float wind_direction;        // Direction du vent (degrés)
  
  // === G-MÈTRE ===
  float g_force;               // Force G actuelle
  float g_force_max;           // Force G max
  float g_force_min;           // Force G min
  
  // === GPS ===
  float latitude;              // Latitude (degrés)
  float longitude;             // Longitude (degrés)
  float latitude_takeoff;      // Latitude décollage
  float longitude_takeoff;     // Longitude décollage
  uint8_t satellites;          // Nombre de satellites
  bool gps_fix;                // Fix GPS valide?
  
  // === PRESSION & TEMPÉRATURE ===
  float pressure;              // Pression atmosphérique (hPa)
  float temperature;           // Température (°C)
  float qnh_local;             // QNH local réglé (hPa)
  
  // === BATTERIE ===
  float battery_voltage;       // Tension batterie (V)
  float battery_percent;       // Pourcentage batterie (%)
  
  // === TEMPS ===
  uint32_t flight_time;        // Temps de vol (secondes)
  uint32_t flight_start_time;  // Heure de début de vol (timestamp)
  bool flight_started;         // Vol démarré?
  
  // === STATISTIQUES ===
  uint32_t last_update_ms;     // Dernier update (millis)
};

// =============================================================================
// VARIABLES GLOBALES
// =============================================================================

static FlightData flight_data;
static SemaphoreHandle_t flight_data_mutex = NULL;

// =============================================================================
// FONCTIONS THREAD-SAFE
// =============================================================================

/**
 * @brief Initialise la structure de données et le mutex
 */
bool flight_data_init() {
  // Créer le mutex
  flight_data_mutex = xSemaphoreCreateMutex();
  if (flight_data_mutex == NULL) {
    return false;
  }
  
  // Initialiser toutes les valeurs à zéro/invalide
  memset(&flight_data, 0, sizeof(FlightData));
  
  flight_data.qnh_local = 1013.25f;  // QNH standard par défaut
  flight_data.gps_fix = false;
  flight_data.flight_started = false;
  
  return true;
}

/**
 * @brief Lock pour écriture (tâche capteurs)
 * @param timeout_ms Timeout en ms
 * @return true si lock acquis
 */
bool flight_data_lock(uint32_t timeout_ms = 100) {
  return xSemaphoreTake(flight_data_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

/**
 * @brief Unlock après écriture
 */
void flight_data_unlock() {
  xSemaphoreGive(flight_data_mutex);
}

/**
 * @brief Copie thread-safe des données (pour UI)
 * @param dest Structure destination
 * @return true si copie réussie
 */
bool flight_data_copy(FlightData* dest) {
  if (xSemaphoreTake(flight_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    memcpy(dest, &flight_data, sizeof(FlightData));
    xSemaphoreGive(flight_data_mutex);
    return true;
  }
  return false;
}

/**
 * @brief Lecture thread-safe d'une seule valeur (exemple: altitude)
 */
float flight_data_get_altitude() {
  float value = 0;
  if (xSemaphoreTake(flight_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    value = flight_data.altitude_qne;
    xSemaphoreGive(flight_data_mutex);
  }
  return value;
}

/**
 * @brief Lecture thread-safe du vario
 */
float flight_data_get_vario() {
  float value = 0;
  if (xSemaphoreTake(flight_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    value = flight_data.vario;
    xSemaphoreGive(flight_data_mutex);
  }
  return value;
}

#endif  // FLIGHT_DATA_H