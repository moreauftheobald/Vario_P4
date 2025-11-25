/**
 * @file flight_calc.h
 * @brief Calculs des paramètres de vol
 * 
 * Fonctions pour calculer:
 * - Altitudes QNH, QFE
 * - Vario intégré/moyenné
 * - Finesse
 * - Distances
 * - Vent
 * - Détection thermique
 * - etc.
 */

#ifndef FLIGHT_CALC_H
#define FLIGHT_CALC_H

#include <Arduino.h>
#include <math.h>
#include "flight_data.h"

// =============================================================================
// CONSTANTES
// =============================================================================
#define EARTH_RADIUS_M 6371000.0f  // Rayon de la Terre en mètres
#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

// Seuils de détection
#define THERMAL_ENTRY_VARIO 0.5f   // Vario > 0.5 m/s pour entrer en thermique
#define THERMAL_EXIT_VARIO 0.2f    // Vario < 0.2 m/s pour sortir
#define FLIGHT_START_VARIO 1.0f    // Vario > 1 m/s pour détecter décollage
#define FLIGHT_START_SPEED 10.0f   // Vitesse > 10 km/h pour détecter décollage

// Historique pour calculs moyennés
#define VARIO_HISTORY_SIZE 150     // 3 secondes à 50Hz
#define GLIDE_HISTORY_SIZE 1500    // 30 secondes à 50Hz

static float vario_history[VARIO_HISTORY_SIZE];
static uint16_t vario_history_idx = 0;
static float glide_history[GLIDE_HISTORY_SIZE];
static uint16_t glide_history_idx = 0;

// =============================================================================
// CALCULS ALTITUDES
// =============================================================================

/**
 * @brief Calcule altitude QNH depuis pression
 * @param pressure Pression mesurée (hPa)
 * @param qnh QNH local (hPa)
 * @return Altitude QNH (m)
 */
float calc_altitude_qnh(float pressure, float qnh) {
  return 44330.0f * (1.0f - powf(pressure / qnh, 0.1903f));
}

/**
 * @brief Calcule altitude QFE (hauteur sol)
 * @param altitude_current Altitude actuelle (m)
 * @param altitude_takeoff Altitude décollage (m)
 * @return Hauteur sol (m)
 */
float calc_altitude_qfe(float altitude_current, float altitude_takeoff) {
  return altitude_current - altitude_takeoff;
}

// =============================================================================
// CALCULS VARIO
// =============================================================================

/**
 * @brief Calcule vario intégré sur 3 secondes
 * @param vario_instant Vario instantané (m/s)
 * @return Vario intégré (m/s)
 */
float calc_vario_integrated(float vario_instant) {
  // Ajouter à l'historique
  vario_history[vario_history_idx] = vario_instant;
  vario_history_idx = (vario_history_idx + 1) % VARIO_HISTORY_SIZE;
  
  // Calculer moyenne
  float sum = 0;
  for (uint16_t i = 0; i < VARIO_HISTORY_SIZE; i++) {
    sum += vario_history[i];
  }
  return sum / VARIO_HISTORY_SIZE;
}

// =============================================================================
// CALCULS DISTANCES
// =============================================================================

/**
 * @brief Calcule distance entre deux points GPS (Haversine)
 * @param lat1, lon1 Point 1 (degrés)
 * @param lat2, lon2 Point 2 (degrés)
 * @return Distance (m)
 */
float calc_distance_gps(float lat1, float lon1, float lat2, float lon2) {
  float dLat = (lat2 - lat1) * DEG_TO_RAD;
  float dLon = (lon2 - lon1) * DEG_TO_RAD;
  
  float a = sinf(dLat / 2) * sinf(dLat / 2) +
            cosf(lat1 * DEG_TO_RAD) * cosf(lat2 * DEG_TO_RAD) *
            sinf(dLon / 2) * sinf(dLon / 2);
  
  float c = 2 * atan2f(sqrtf(a), sqrtf(1 - a));
  
  return EARTH_RADIUS_M * c;
}

/**
 * @brief Calcule cap entre deux points GPS
 * @param lat1, lon1 Point départ (degrés)
 * @param lat2, lon2 Point arrivée (degrés)
 * @return Cap (degrés, 0-360)
 */
float calc_bearing_gps(float lat1, float lon1, float lat2, float lon2) {
  float dLon = (lon2 - lon1) * DEG_TO_RAD;
  float y = sinf(dLon) * cosf(lat2 * DEG_TO_RAD);
  float x = cosf(lat1 * DEG_TO_RAD) * sinf(lat2 * DEG_TO_RAD) -
            sinf(lat1 * DEG_TO_RAD) * cosf(lat2 * DEG_TO_RAD) * cosf(dLon);
  
  float bearing = atan2f(y, x) * RAD_TO_DEG;
  return fmodf(bearing + 360.0f, 360.0f);  // Normaliser 0-360
}

// =============================================================================
// CALCULS FINESSE
// =============================================================================

/**
 * @brief Calcule finesse instantanée
 * @param speed_horiz Vitesse horizontale (m/s)
 * @param vario Vitesse verticale (m/s)
 * @return Finesse (sans unité)
 */
float calc_glide_ratio(float speed_horiz, float vario) {
  if (fabsf(vario) < 0.1f) return 99.9f;  // Eviter division par 0
  
  float glide = fabsf(speed_horiz / vario);
  
  // Limiter à 99.9 pour affichage
  if (glide > 99.9f) glide = 99.9f;
  if (glide < 0) glide = 0;
  
  return glide;
}

/**
 * @brief Calcule finesse moyennée sur 30s
 * @param glide_instant Finesse instantanée
 * @return Finesse moyenne (sans unité)
 */
float calc_glide_ratio_avg(float glide_instant) {
  // Ajouter à l'historique
  glide_history[glide_history_idx] = glide_instant;
  glide_history_idx = (glide_history_idx + 1) % GLIDE_HISTORY_SIZE;
  
  // Calculer moyenne (ignorer valeurs > 50 = bruits)
  float sum = 0;
  uint16_t count = 0;
  for (uint16_t i = 0; i < GLIDE_HISTORY_SIZE; i++) {
    if (glide_history[i] < 50.0f) {
      sum += glide_history[i];
      count++;
    }
  }
  
  return (count > 0) ? (sum / count) : 0;
}

// =============================================================================
// CALCULS VENT
// =============================================================================

/**
 * @brief Estime le vent depuis vitesse GPS vs vitesse air
 * @param speed_gps Vitesse GPS (m/s)
 * @param heading_gps Cap GPS (degrés)
 * @param speed_air Vitesse air estimée (m/s)
 * @param wind_speed Sortie: vitesse vent (m/s)
 * @param wind_dir Sortie: direction vent (degrés)
 */
void calc_wind(float speed_gps, float heading_gps, float speed_air,
               float* wind_speed, float* wind_dir) {
  // Calcul simplifié (méthode du vecteur)
  // TODO: Implémenter algorithme plus robuste sur plusieurs points
  
  float heading_rad = heading_gps * DEG_TO_RAD;
  
  // Composantes GPS
  float vx_gps = speed_gps * sinf(heading_rad);
  float vy_gps = speed_gps * cosf(heading_rad);
  
  // Composantes air (assumé même cap)
  float vx_air = speed_air * sinf(heading_rad);
  float vy_air = speed_air * cosf(heading_rad);
  
  // Vent = GPS - Air
  float wx = vx_gps - vx_air;
  float wy = vy_gps - vy_air;
  
  *wind_speed = sqrtf(wx*wx + wy*wy);
  *wind_dir = fmodf(atan2f(wx, wy) * RAD_TO_DEG + 360.0f, 360.0f);
}

// =============================================================================
// DÉTECTION THERMIQUE
// =============================================================================

/**
 * @brief Détecte entrée/sortie de thermique
 * @param vario Vario actuel (m/s)
 * @param in_thermal État actuel
 * @return Nouvel état
 */
bool detect_thermal(float vario, bool in_thermal) {
  if (!in_thermal && vario > THERMAL_ENTRY_VARIO) {
    return true;  // Entrée en thermique
  }
  
  if (in_thermal && vario < THERMAL_EXIT_VARIO) {
    return false;  // Sortie de thermique
  }
  
  return in_thermal;  // Pas de changement
}

// =============================================================================
// DÉTECTION DÉBUT DE VOL
// =============================================================================

/**
 * @brief Détecte le début du vol
 * @param vario Vario (m/s)
 * @param speed_gps Vitesse GPS (km/h)
 * @param altitude Altitude (m)
 * @param flight_started État actuel
 * @return true si vol démarré
 */
bool detect_flight_start(float vario, float speed_gps, float altitude, 
                         bool flight_started) {
  if (flight_started) return true;  // Déjà démarré
  
  // Décollage détecté si vario > 1 m/s OU vitesse > 10 km/h
  if (vario > FLIGHT_START_VARIO || speed_gps > FLIGHT_START_SPEED) {
    return true;
  }
  
  return false;
}

// =============================================================================
// CALCUL G-FORCE
// =============================================================================

/**
 * @brief Calcule la force G depuis l'accélération
 * @param acc_x, acc_y, acc_z Accélérations (m/s²)
 * @return Force G
 */
float calc_g_force(float acc_x, float acc_y, float acc_z) {
  float acc_total = sqrtf(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
  return acc_total / 9.81f;
}

#endif  // FLIGHT_CALC_H