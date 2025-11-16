/**
 * @file flight_data.h
 * @brief Structure globale des données de vol
 * 
 * Contient toutes les données calculées par task_flight,
 * accessibles en lecture par les autres tâches via mutex.
 * 
 * @author Franck Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#ifndef FLIGHT_DATA_H
#define FLIGHT_DATA_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Structure complète des données de vol
 */
typedef struct {
    // === Timestamp ===
    uint32_t timestamp_ms;          // Timestamp données (millis)
    
    // === Altitudes ===
    float altitude_gps;             // Altitude GPS (m)
    float altitude_qne;             // Altitude QNE 1013.25 hPa (m)
    float altitude_qnh;             // Altitude QNH pression locale (m)
    float altitude_agl;             // Hauteur sol - QFE (m)
    float altitude_takeoff;         // Altitude décollage (m)
    
    // === Vario ===
    float vario;                    // Vario instantané (m/s)
    float vario_integrated;         // Vario intégré (moyenne mobile)
    float vario_avg_10s;            // Vario moyen 10 secondes
    float vario_avg_30s;            // Vario moyen 30 secondes
    
    // === Gains ===
    float gain_thermal;             // Gain dans le thermique actuel (m)
    float gain_flight;              // Gain depuis décollage (m)
    
    // === Vitesses ===
    float speed_ground;             // Vitesse sol GPS (km/h)
    float speed_air;                // Vitesse air estimée (km/h)
    float speed_vertical;           // Vitesse verticale (m/s) = vario
    
    // === Position GPS ===
    float latitude;                 // Latitude (degrés décimaux)
    float longitude;                // Longitude (degrés décimaux)
    float heading;                  // Cap GPS (degrés, 0-360)
    uint8_t satellites;             // Nombre de satellites
    bool gps_fix;                   // Fix GPS valide
    float hdop;                     // Dilution précision horizontale
    
    // === Distances ===
    float distance_total;           // Distance totale parcourue (km)
    float distance_takeoff;         // Distance depuis décollage (km)
    float bearing_takeoff;          // Cap vers décollage (degrés)
    
    // === Finesse ===
    float glide_ratio;              // Finesse instantanée
    float glide_ratio_avg;          // Finesse moyenne
    
    // === Vent (estimation) ===
    float wind_speed;               // Vitesse vent estimée (km/h)
    float wind_direction;           // Direction vent (degrés)
    
    // === G-mètre ===
    float g_force;                  // Force G actuelle
    float g_force_max;              // Force G max du vol
    float g_force_min;              // Force G min du vol
    
    // === Temps ===
    uint32_t time_flight;           // Temps de vol (secondes)
    uint8_t hour;                   // Heure GPS
    uint8_t minute;                 // Minute GPS
    uint8_t second;                 // Seconde GPS
    
    // === Pression / Température ===
    float pressure;                 // Pression atmosphérique (hPa)
    float temperature;              // Température (°C)
    float qnh;                      // QNH configuré (hPa)
    
    // === Orientation (quaternions) ===
    struct {
        float w, x, y, z;
    } quaternion;                   // Quaternion orientation
    
    // === Angles Euler (informatif) ===
    float roll;                     // Roulis (degrés)
    float pitch;                    // Tangage (degrés)
    float yaw;                      // Lacet (degrés)
    
    // === Accélérations ===
    float accel_vertical;           // Accélération verticale (m/s²)
    float accel_forward;            // Accélération avant/arrière (m/s²)
    float accel_lateral;            // Accélération latérale (m/s²)
    
    // === État vol ===
    bool in_flight;                 // En vol (true) ou au sol (false)
    bool in_thermal;                // Dans un thermique
    uint32_t takeoff_time;          // Timestamp décollage (millis)
    
    // === Statistiques vol ===
    float altitude_max;             // Altitude max du vol (m)
    float altitude_min;             // Altitude min du vol (m)
    float vario_max;                // Vario max du vol (m/s)
    float vario_min;                // Vario min du vol (m/s)
    
    // === Qualité données ===
    uint8_t data_quality;           // Qualité globale données (0-100)
    bool kalman_converged;          // Kalman convergé
    
} flight_data_t;

// Variable globale des données de vol
extern flight_data_t g_flight_data;

#endif // FLIGHT_DATA_H