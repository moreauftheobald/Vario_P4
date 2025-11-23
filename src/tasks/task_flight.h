/**
 * @file task_flight.h
 * @brief Tâche principale de calcul variomètre
 * 
 * Gère :
 * - Lecture capteurs (IMU 200Hz, BMP5 100Hz, GPS 2Hz, Battery 1Hz)
 * - Fusion IMU (Madgwick 200Hz)
 * - Kalman altitude/vario (100Hz)
 * - Calculs paramètres de vol (2Hz)
 * - Health monitoring capteurs
 * 
 * Fréquence base : 200 Hz (5ms)
 * Core : 1 (dédié calculs temps réel)
 * Priorité : 5 (haute)
 * 
 * @author Franck Moreau
 * @date 2025-11-23
 * @version 1.0
 * 
 * LOG DOCUMENTATION - MODULE: FLIGHT
 * [ERROR]
 *   - "Task flight creation failed" : Échec création tâche
 *   - "Sensor %s unhealthy" : Capteur défaillant
 * 
 * [WARNING]
 *   - "Sensor %s timeout" : Capteur ne répond plus
 *   - "Kalman not converged" : Filtre pas encore stabilisé
 * 
 * [INFO]
 *   - "Task flight started" : Tâche démarrée
 *   - "Takeoff detected" : Décollage détecté
 *   - "Landing detected" : Atterrissage détecté
 * 
 * [VERBOSE]
 *   - "Flight update: alt=%.1f vario=%.2f" : Mise à jour données
 */

#ifndef TASK_FLIGHT_H
#define TASK_FLIGHT_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "src/data/flight_data.h"
#include "src/core/madgwick_filter/madgwick_filter.h"
#include "src/core/kalman_filter/kalman_filter.h"
#include "src/system/sensor_init/sensor_init.h"
#include "src/system/logger/logger.h"
#include "config/config.h"

// =============================================================================
#define TASK_FLIGHT_MONITOR_ENABLED 1    // Activer monitoring (0=off, 1=on)
#define MONITOR_PRINT_INTERVAL_MS 10000  // Afficher stats toutes les 10s

typedef struct {
  // Temps d'exécution (µs)
  uint32_t cycle_time_us;      // Temps du dernier cycle
  uint32_t cycle_time_max_us;  // Temps max observé
  uint32_t cycle_time_min_us;  // Temps min observé
  uint32_t cycle_time_avg_us;  // Temps moyen

  // Temps par fonction (µs)
  uint32_t imu_time_us;
  uint32_t bmp5_time_us;
  uint32_t kalman_time_us;
  uint32_t gps_time_us;
  uint32_t calculs_time_us;

  // Overruns
  uint32_t overrun_count;   // Nombre de cycles > 5ms
  uint32_t overrun_max_us;  // Pire overrun observé

  // Statistiques
  uint32_t total_cycles;   // Nombre total de cycles
  uint64_t total_time_us;  // Temps cumulé (pour moyenne)

  // Dernière impression
  uint32_t last_print_ms;

} task_flight_monitor_t;

static task_flight_monitor_t monitor = { 0 };

// =============================================================================
// CONFIGURATION TÂCHE
// =============================================================================
#define TASK_FLIGHT_FREQ_HZ 200      // Fréquence de base (Hz)
#define TASK_FLIGHT_PERIOD_MS 5      // Période (ms)
#define TASK_FLIGHT_STACK_SIZE 8192  // Stack (mots 32-bit)
#define TASK_FLIGHT_PRIORITY 5       // Priorité haute
#define TASK_FLIGHT_CORE 1           // Core 1 dédié

// Diviseurs de fréquence
#define DIV_IMU 1        // 200 Hz
#define DIV_MADGWICK 1   // 200 Hz
#define DIV_KALMAN 2     // 100 Hz
#define DIV_BMP5 2       // 100 Hz
#define DIV_GPS 100      // 2 Hz
#define DIV_BATTERY 200  // 1 Hz
#define DIV_CALCULS 100  // 2 Hz

// Seuils détection vol
#define TAKEOFF_SPEED_THRESHOLD 10.0f  // km/h
#define TAKEOFF_VARIO_THRESHOLD 0.5f   // m/s
#define LANDING_SPEED_THRESHOLD 3.0f   // km/h
#define LANDING_TIME_MS 5000           // 5s sous seuil

// =============================================================================
// FONCTIONS PUBLIQUES
// =============================================================================

/**
 * @brief Initialise le monitoring
 */
static void monitor_init() {
  memset(&monitor, 0, sizeof(task_flight_monitor_t));
  monitor.cycle_time_min_us = UINT32_MAX;
  monitor.last_print_ms = millis();
}

/**
 * @brief Démarre la mesure d'un cycle
 */
static inline uint32_t monitor_cycle_start() {
  return micros();
}

/**
 * @brief Termine la mesure d'un cycle
 */
static void monitor_cycle_end(uint32_t start_us) {
  uint32_t elapsed_us = micros() - start_us;

  monitor.cycle_time_us = elapsed_us;
  monitor.total_cycles++;
  monitor.total_time_us += elapsed_us;

  // Min/Max
  if (elapsed_us > monitor.cycle_time_max_us) {
    monitor.cycle_time_max_us = elapsed_us;
  }
  if (elapsed_us < monitor.cycle_time_min_us) {
    monitor.cycle_time_min_us = elapsed_us;
  }

  // Moyenne
  monitor.cycle_time_avg_us = (uint32_t)(monitor.total_time_us / monitor.total_cycles);

  // Overrun (> 5000µs = 5ms)
  if (elapsed_us > (TASK_SENSORS_PERIOD_MS * 1000)) {
    monitor.overrun_count++;
    if (elapsed_us > monitor.overrun_max_us) {
      monitor.overrun_max_us = elapsed_us;
    }
  }
}

/**
 * @brief Mesure le temps d'une fonction
 */
static inline uint32_t monitor_func_time(uint32_t start_us) {
  return micros() - start_us;
}

/**
 * @brief Affiche les statistiques
 */
static void monitor_print_stats() {
  uint32_t now = millis();

  if (now - monitor.last_print_ms < MONITOR_PRINT_INTERVAL_MS) {
    return;
  }

  monitor.last_print_ms = now;

  // Calculer CPU usage
  float cpu_usage = (monitor.cycle_time_avg_us * 100.0f) / (TASK_SENSORS_PERIOD_MS * 1000.0f);

  LOG_I(LOG_MODULE_FLIGHT, "");
  LOG_I(LOG_MODULE_FLIGHT, "=== Task Flight Performance ===");
  LOG_I(LOG_MODULE_FLIGHT, "Cycles: %lu (overruns: %lu = %.2f%%)",
        monitor.total_cycles,
        monitor.overrun_count,
        (monitor.overrun_count * 100.0f) / monitor.total_cycles);

  LOG_I(LOG_MODULE_FLIGHT, "Cycle time: min=%lu avg=%lu max=%lu µs",
        monitor.cycle_time_min_us,
        monitor.cycle_time_avg_us,
        monitor.cycle_time_max_us);

  LOG_I(LOG_MODULE_FLIGHT, "CPU usage: %.1f%% (budget: %d µs / %d µs)",
        cpu_usage,
        monitor.cycle_time_avg_us,
        TASK_SENSORS_PERIOD_MS * 1000);

  if (monitor.overrun_count > 0) {
    LOG_W(LOG_MODULE_FLIGHT, "Worst overrun: %lu µs (%.1f ms over budget)",
          monitor.overrun_max_us,
          (monitor.overrun_max_us - TASK_SENSORS_PERIOD_MS * 1000) / 1000.0f);
  }

  LOG_I(LOG_MODULE_FLIGHT, "Function times (last cycle):");
  LOG_I(LOG_MODULE_FLIGHT, "  IMU:     %lu µs", monitor.imu_time_us);
  LOG_I(LOG_MODULE_FLIGHT, "  BMP5:    %lu µs", monitor.bmp5_time_us);
  LOG_I(LOG_MODULE_FLIGHT, "  Kalman:  %lu µs", monitor.kalman_time_us);
  LOG_I(LOG_MODULE_FLIGHT, "  GPS:     %lu µs", monitor.gps_time_us);
  LOG_I(LOG_MODULE_FLIGHT, "  Calculs: %lu µs", monitor.calculs_time_us);
  LOG_I(LOG_MODULE_FLIGHT, "================================");
  LOG_I(LOG_MODULE_FLIGHT, "");
}

/**
 * @brief Démarre la tâche flight
 * 
 * Crée la tâche FreeRTOS et initialise filtres (Madgwick + Kalman).
 * 
 * @return true si succès, false si erreur
 */
bool task_flight_start();

/**
 * @brief Arrête la tâche flight
 */
void task_flight_stop();

/**
 * @brief Vérifie si la tâche est active
 * 
 * @return true si active, false sinon
 */
bool task_flight_is_running();

/**
 * @brief Obtient les données de vol (thread-safe)
 * 
 * Copie les données dans la structure fournie avec protection mutex.
 * 
 * @param[out] data Structure à remplir
 * @return true si succès, false si timeout mutex
 */
bool task_flight_get_data(flight_data_t* data);

/**
 * @brief Force le reset du Kalman
 * 
 * Utile après changement QNH ou altitude manuelle.
 * 
 * @param[in] altitude Nouvelle altitude de référence (m)
 */
void task_flight_reset_kalman(float altitude);

/**
 * @brief Configure le QNH
 * 
 * @param[in] qnh QNH en hPa (ex: 1013.25)
 */
void task_flight_set_qnh(float qnh);

// =============================================================================
// IMPLÉMENTATION (dans .h car tâche FreeRTOS)
// =============================================================================

// Structure cache IMU commune (pour BNO08x ET LSM6DSO32)
typedef struct {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
} imu_data_cache_t;

// Variables privées
static TaskHandle_t task_flight_handle = NULL;
static SemaphoreHandle_t flight_data_mutex = NULL;
static madgwick_filter_t madgwick;
static kalman_filter_t kalman;
static bool task_running = false;

// Dernières lectures capteurs (structure COMMUNE)
static imu_data_cache_t last_imu_data;
static bmp5_data_t last_bmp_data;
static max17048_data_t last_battery_data;

// QNH courant
static float current_qnh = 1013.25f;
static float gyro_offset_x = -0.001119f;
static float gyro_offset_y = -0.001825f;
static float gyro_offset_z = -0.000743f;

// =============================================================================
// FONCTIONS HELPER PRIVÉES
// =============================================================================

/*#if IMU_BNO08XX == 1
/**
 * @brief Lit le BNO08x et met à jour les quaternions
 */
/*static bool read_and_fuse_imu() {
    if (!sensor_imu_ready || !bno08x_dev) return false;
    
    sh2_SensorValue_t event;
    bool data_updated = false;
    int events_read = 0;
    
    // ✅ Lire TOUS les événements disponibles (polling actif)
    // Le BNO08x peut avoir plusieurs rapports en attente
    while (bno08x_dev->getSensorEvent(&event) && events_read < 10) {
        events_read++;
        
        switch (event.sensorId) {
            case SH2_ROTATION_VECTOR:
                // Quaternions (fusion IMU matérielle)
                madgwick.q.w = event.un.rotationVector.real;
                madgwick.q.x = event.un.rotationVector.i;
                madgwick.q.y = event.un.rotationVector.j;
                madgwick.q.z = event.un.rotationVector.k;
                data_updated = true;
                break;
                
            case SH2_LINEAR_ACCELERATION:
                // Accélération linéaire (sans gravité)
                last_imu_data.accel_x = event.un.linearAcceleration.x;
                last_imu_data.accel_y = event.un.linearAcceleration.y;
                last_imu_data.accel_z = event.un.linearAcceleration.z;
                data_updated = true;
                break;
                
            case SH2_GYROSCOPE_CALIBRATED:
                last_imu_data.gyro_x = event.un.gyroscope.x;
                last_imu_data.gyro_y = event.un.gyroscope.y;
                last_imu_data.gyro_z = event.un.gyroscope.z;
                break;
                
            case SH2_ACCELEROMETER:
                // Accéléromètre brut (backup si LINEAR_ACCELERATION pas dispo)
                if (last_imu_data.accel_x == 0 && last_imu_data.accel_y == 0) {
                    last_imu_data.accel_x = event.un.accelerometer.x;
                    last_imu_data.accel_y = event.un.accelerometer.y;
                    last_imu_data.accel_z = event.un.accelerometer.z;
                }
                break;
                
            default:
                break;
        }
    }
    
    // ✅ Log périodique pour debug
    static uint32_t last_debug = 0;
    if (millis() - last_debug > 5000) {
        LOG_I(LOG_MODULE_FLIGHT, "BNO08x: %d events read, updated=%d", events_read, data_updated);
        last_debug = millis();
    }
    
    if (data_updated) {
        g_flight_data.sensors_health.imu_healthy = true;
        g_flight_data.sensors_health.imu_error_count = 0;
        g_flight_data.sensors_health.last_imu_success = millis();
        return true;
    }
    
    // ✅ Si aucune donnée après plusieurs cycles, incrémenter erreur
    static uint32_t last_data = millis();
    if (data_updated) {
        last_data = millis();
    } else if (millis() - last_data > 1000) {
        g_flight_data.sensors_health.imu_error_count++;
    }
    
    return false;
}
#endif*/

/**
 * @brief Lit le BMP5
 * 
 * @return true si succès
 */
static bool read_bmp5() {
  if (!sensor_bmp5_ready) return false;

  if (!BMP5_read(&bmp5_dev, &last_bmp_data, current_qnh)) {
    g_flight_data.sensors_health.baro_error_count++;
    return false;
  }

  g_flight_data.sensors_health.baro_healthy = true;
  g_flight_data.sensors_health.baro_error_count = 0;
  g_flight_data.sensors_health.last_baro_success = millis();

  return true;
}

/**
 * @brief Lit le GPS
 * 
 * @return true si nouvelles données parsées
 */
static bool read_gps() {
  if (!sensor_gps_ready) return false;

  // Utiliser la fonction existante de sensor_init
  bool new_data = sensor_read_gps();

  if (new_data) {
    // Copier données GPS dans flight_data
    g_flight_data.gps_fix = gps_data.fix;
    g_flight_data.satellites = gps_data.satellites;
    g_flight_data.hdop = gps_data.hdop;

    if (gps_data.fix) {
      g_flight_data.latitude = gps_data.latitude;
      g_flight_data.longitude = gps_data.longitude;
      g_flight_data.altitude_gps = gps_data.altitude;
      g_flight_data.speed_ground = gps_data.speed * 1.852f;
      g_flight_data.heading = gps_data.course;

      // Heure
      g_flight_data.hour = gps_data.hour;
      g_flight_data.minute = gps_data.minute;
      g_flight_data.second = gps_data.second;

      // Date
      g_flight_data.day = gps_data.day;
      g_flight_data.month = gps_data.month;
      g_flight_data.year = gps_data.year;
    }

    // Mettre à jour health
    g_flight_data.sensors_health.gps_healthy = true;
    g_flight_data.sensors_health.gps_error_count = 0;
    g_flight_data.sensors_health.last_gps_success = millis();
  }

  return new_data;
}

/**
 * @brief Lit la batterie
 * 
 * @return true si succès
 */
static bool read_battery() {
  if (!sensor_battery_ready) return false;

  if (!MAX17048_read(&max17048_dev, &last_battery_data)) {
    return false;
  }

  // Copier dans flight_data (pas besoin de mutex ici, c'est la task qui écrit)
  // TODO: Ajouter champs batterie dans flight_data.h

  return true;
}

/**
 * @brief Update Kalman sélectif (baro optionnel)
 */
static void update_kalman_selective(bool update_baro) {
  // Accélération verticale
  float accel_vertical = madgwick_get_vertical_accel(&madgwick,
                                                     last_imu_data.accel_x,
                                                     last_imu_data.accel_y,
                                                     last_imu_data.accel_z);

  // Prédiction (toujours à 200 Hz)
  float dt = TASK_SENSORS_PERIOD_MS / 1000.0f;
  kalman_predict(&kalman, accel_vertical, dt);

  // Correction baro (seulement si demandé = 5 Hz)
  if (update_baro) {
    kalman_update_baro(&kalman, last_bmp_data.altitude);
  }

  // Récupérer résultats
  g_flight_data.altitude_qnh = kalman_get_altitude(&kalman);
  g_flight_data.vario = kalman_get_vario(&kalman);
  g_flight_data.kalman_converged = kalman_is_converged(&kalman);
}

/**
 * @brief Prédiction Kalman seule (pas de correction)
 */
static void kalman_predict_only() {
  // Accélération verticale
  float accel_vertical = madgwick_get_vertical_accel(&madgwick,
                                                     last_imu_data.accel_x,
                                                     last_imu_data.accel_y,
                                                     last_imu_data.accel_z);

  // Prédiction uniquement
  float dt = TASK_SENSORS_PERIOD_MS / 1000.0f;
  kalman_predict(&kalman, accel_vertical, dt);

  // Récupérer résultats (pas de correction)
  g_flight_data.altitude_qnh = kalman_get_altitude(&kalman);
  g_flight_data.vario = kalman_get_vario(&kalman);
}

/**
 * @brief Met à jour le Kalman (prédiction + correction)
 */
static void update_kalman() {
  // Accélération verticale depuis Madgwick
  float accel_vertical = madgwick_get_vertical_accel(&madgwick,
                                                     last_imu_data.accel_x,
                                                     last_imu_data.accel_y,
                                                     last_imu_data.accel_z);

  // Prédiction (avec accélération IMU)
  float dt = TASK_FLIGHT_PERIOD_MS / 1000.0f;
  kalman_predict(&kalman, accel_vertical, dt);

  // Correction baromètre (si nouvelle lecture)
  static uint32_t last_baro_update = 0;
  if (millis() - last_baro_update >= (1000 / (TASK_FLIGHT_FREQ_HZ / DIV_BMP5))) {
    last_baro_update = millis();
    kalman_update_baro(&kalman, last_bmp_data.altitude);
  }

  // Récupérer résultats Kalman
  g_flight_data.altitude_qnh = kalman_get_altitude(&kalman);
  g_flight_data.vario = kalman_get_vario(&kalman);
  g_flight_data.kalman_converged = kalman_is_converged(&kalman);
}

/**
 * @brief Calcule les paramètres de vol dérivés (2Hz)
 */
static void calculate_flight_params() {
  // Altitudes
  g_flight_data.altitude_qne = last_bmp_data.altitude;  // QNE = 1013.25

  // AGL (hauteur sol) = altitude actuelle - altitude décollage
  if (g_flight_data.in_flight) {
    g_flight_data.altitude_agl = g_flight_data.altitude_qnh - g_flight_data.altitude_takeoff;
  } else {
    g_flight_data.altitude_agl = 0.0f;
  }

  // Statistiques altitude
  if (g_flight_data.altitude_qnh > g_flight_data.altitude_max) {
    g_flight_data.altitude_max = g_flight_data.altitude_qnh;
  }
  if (g_flight_data.altitude_qnh < g_flight_data.altitude_min) {
    g_flight_data.altitude_min = g_flight_data.altitude_qnh;
  }

  // Statistiques vario
  if (g_flight_data.vario > g_flight_data.vario_max) {
    g_flight_data.vario_max = g_flight_data.vario;
  }
  if (g_flight_data.vario < g_flight_data.vario_min) {
    g_flight_data.vario_min = g_flight_data.vario;
  }

  // Angles d'Euler depuis quaternion
  euler_t euler;
  madgwick_quaternion_to_euler(&madgwick.q, &euler);
  g_flight_data.roll = euler.roll;
  g_flight_data.pitch = euler.pitch;
  g_flight_data.yaw = euler.yaw;

  // Copier quaternion
  g_flight_data.quaternion.w = madgwick.q.w;
  g_flight_data.quaternion.x = madgwick.q.x;
  g_flight_data.quaternion.y = madgwick.q.y;
  g_flight_data.quaternion.z = madgwick.q.z;

  // Pression/température
  g_flight_data.pressure = last_bmp_data.pressure;
  g_flight_data.temperature = last_bmp_data.temperature;
  g_flight_data.qnh = current_qnh;

  // Accélérations (TODO: calculer dans référentiel terrestre)
  g_flight_data.accel_vertical = madgwick_get_vertical_accel(&madgwick,
                                                             last_imu_data.accel_x,
                                                             last_imu_data.accel_y,
                                                             last_imu_data.accel_z);

  // G-force
  float g_total = sqrtf(last_imu_data.accel_x * last_imu_data.accel_x + last_imu_data.accel_y * last_imu_data.accel_y + last_imu_data.accel_z * last_imu_data.accel_z) / 9.80665f;
  g_flight_data.g_force = g_total;

  if (g_total > g_flight_data.g_force_max) {
    g_flight_data.g_force_max = g_total;
  }
  if (g_total < g_flight_data.g_force_min) {
    g_flight_data.g_force_min = g_total;
  }

  // Détection décollage/atterrissage
  static uint32_t landing_timer = 0;

  if (!g_flight_data.in_flight) {
    // Détection décollage
    if (g_flight_data.speed_ground > TAKEOFF_SPEED_THRESHOLD || fabsf(g_flight_data.vario) > TAKEOFF_VARIO_THRESHOLD) {
      g_flight_data.in_flight = true;
      g_flight_data.takeoff_time = millis();
      g_flight_data.altitude_takeoff = g_flight_data.altitude_qnh;
      LOG_I(LOG_MODULE_FLIGHT, "Takeoff detected at %.1fm", g_flight_data.altitude_takeoff);
    }
  } else {
    // Détection atterrissage
    if (g_flight_data.speed_ground < LANDING_SPEED_THRESHOLD && fabsf(g_flight_data.vario) < 0.2f) {

      if (landing_timer == 0) {
        landing_timer = millis();
      } else if (millis() - landing_timer > LANDING_TIME_MS) {
        g_flight_data.in_flight = false;
        landing_timer = 0;
        LOG_I(LOG_MODULE_FLIGHT, "Landing detected");
      }
    } else {
      landing_timer = 0;
    }

    // Temps de vol
    g_flight_data.time_flight = (millis() - g_flight_data.takeoff_time) / 1000;
  }

  // Qualité données
  uint8_t quality = 0;
  if (g_flight_data.sensors_health.imu_healthy) quality += 33;
  if (g_flight_data.sensors_health.baro_healthy) quality += 33;
  if (g_flight_data.sensors_health.gps_healthy) quality += 34;
  g_flight_data.data_quality = quality;

  // Timestamp
  g_flight_data.timestamp_ms = millis();

  LOG_V(LOG_MODULE_FLIGHT, "Flight update: alt=%.1f vario=%.2f speed=%.1f",
        g_flight_data.altitude_qnh, g_flight_data.vario, g_flight_data.speed_ground);
}

/**
 * @brief Vérifie la santé des capteurs
 */
static void check_sensors_health() {
  uint32_t now = millis();

  // IMU
  if (now - g_flight_data.sensors_health.last_imu_success > SENSOR_TIMEOUT_MS) {
    if (g_flight_data.sensors_health.imu_healthy) {
      g_flight_data.sensors_health.imu_healthy = false;
      LOG_E(LOG_MODULE_FLIGHT, "Sensor IMU unhealthy (timeout)");
    }
  }

  // Baro
  if (now - g_flight_data.sensors_health.last_baro_success > SENSOR_TIMEOUT_MS) {
    if (g_flight_data.sensors_health.baro_healthy) {
      g_flight_data.sensors_health.baro_healthy = false;
      LOG_E(LOG_MODULE_FLIGHT, "Sensor BMP5 unhealthy (timeout)");
    }
  }

  // GPS
  if (now - g_flight_data.sensors_health.last_gps_success > SENSOR_TIMEOUT_MS) {
    if (g_flight_data.sensors_health.gps_healthy) {
      g_flight_data.sensors_health.gps_healthy = false;
      LOG_W(LOG_MODULE_FLIGHT, "Sensor GPS unhealthy (timeout)");
    }
  }
}

// =============================================================================
// TÂCHE FREERTOS
// =============================================================================

static void task_flight_loop(void* parameter) {
  TickType_t last_wake_time = xTaskGetTickCount();
  uint32_t cycle_count = 0;

  // ✅ Timers pour UPDATE Kalman (indépendants de la lecture)
  static uint32_t last_baro_update = 0;
  static uint32_t last_imu_update = 0;

  LOG_I(LOG_MODULE_FLIGHT, "Task flight started @ %d Hz", TASK_SENSORS_FREQ_HZ);

#if TASK_FLIGHT_MONITOR_ENABLED
  monitor_init();
#endif

  while (task_running) {
#if TASK_FLIGHT_MONITOR_ENABLED
    uint32_t cycle_start = monitor_cycle_start();
    uint32_t func_start;
#endif

    uint32_t now = millis();

// ========== LECTURE IMU + MADGWICK (200 Hz) ==========
#if TASK_FLIGHT_MONITOR_ENABLED
    func_start = micros();
#endif

    //read_and_fuse_imu();  // Lecture 200 Hz (toujours)

#if TASK_FLIGHT_MONITOR_ENABLED
    monitor.imu_time_us = monitor_func_time(func_start);
#endif

    // ========== LECTURE BMP5 (100 Hz) ==========
    if (cycle_count % FREQ_BMP5_DIVIDER == 0) {
#if TASK_FLIGHT_MONITOR_ENABLED
      func_start = micros();
#endif

      read_bmp5();  // Lecture 100 Hz (toujours)

#if TASK_FLIGHT_MONITOR_ENABLED
      monitor.bmp5_time_us = monitor_func_time(func_start);
#endif
    }

// ========== KALMAN UPDATE (configurable) ==========
#if TASK_FLIGHT_MONITOR_ENABLED
    func_start = micros();
#endif

    // ✅ UPDATE IMU dans Kalman : 50 Hz (toutes les 20ms)
    bool imu_update_needed = (now - last_imu_update >= 20);

    // ✅ UPDATE BARO dans Kalman : 5 Hz (toutes les 200ms)
    bool baro_update_needed = (now - last_baro_update >= 200);

    if (imu_update_needed || baro_update_needed) {
      update_kalman_selective(baro_update_needed);

      if (imu_update_needed) last_imu_update = now;
      if (baro_update_needed) last_baro_update = now;
    } else {
      // Prédiction seule (sans correction)
      kalman_predict_only();
    }

#if TASK_FLIGHT_MONITOR_ENABLED
    monitor.kalman_time_us = monitor_func_time(func_start);
#endif

    // ========== LECTURE GPS (2 Hz) ==========
    if (cycle_count % FREQ_GPS_DIVIDER == 0) {
#if TASK_FLIGHT_MONITOR_ENABLED
      func_start = micros();
#endif

      read_gps();

#if TASK_FLIGHT_MONITOR_ENABLED
      monitor.gps_time_us = monitor_func_time(func_start);
#endif
    }

    // ========== LECTURE BATTERIE (2 Hz) ==========
    if (cycle_count % FREQ_BATTERY_DIVIDER == 0) {
      read_battery();
    }

    // ========== CALCULS FLIGHT DATA (2 Hz) ==========
    if (cycle_count % FREQ_FLIGHT_CALCULS_DIVIDER == 0) {
#if TASK_FLIGHT_MONITOR_ENABLED
      func_start = micros();
#endif

      calculate_flight_params();
      check_sensors_health();

#if TASK_FLIGHT_MONITOR_ENABLED
      monitor.calculs_time_us = monitor_func_time(func_start);
#endif
    }

#if TASK_FLIGHT_MONITOR_ENABLED
    monitor_cycle_end(cycle_start);
    monitor_print_stats();
#endif

    cycle_count++;

    // Attendre prochain cycle (5ms = 200 Hz)
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_SENSORS_PERIOD_MS));
  }

  LOG_I(LOG_MODULE_FLIGHT, "Task flight stopped");
  vTaskDelete(NULL);
}

// =============================================================================
// FONCTIONS PUBLIQUES (IMPLÉMENTATION)
// =============================================================================
bool task_flight_start() {
  if (task_running) {
    LOG_W(LOG_MODULE_FLIGHT, "Task already running");
    return true;
  }

  LOG_I(LOG_MODULE_FLIGHT, "Starting task flight...");

  // Créer mutex
  flight_data_mutex = xSemaphoreCreateMutex();
  if (!flight_data_mutex) {
    LOG_E(LOG_MODULE_FLIGHT, "Failed to create mutex");
    return false;
  }

  // Init Madgwick
  madgwick_init(&madgwick, TASK_SENSORS_FREQ_HZ, 0.1f);

/*#if IMU_BNO08XX == 1
  // BNO08x : quaternions hardware
  LOG_I(LOG_MODULE_FLIGHT, "Using BNO08x - hardware fusion");

  sh2_SensorValue_t event;
  uint32_t wait_start = millis();
  while (millis() - wait_start < 1000) {
    if (bno08x_dev->getSensorEvent(&event)) {
      if (event.sensorId == SH2_ROTATION_VECTOR) {
        madgwick.q.w = event.un.rotationVector.real;
        madgwick.q.x = event.un.rotationVector.i;
        madgwick.q.y = event.un.rotationVector.j;
        madgwick.q.z = event.un.rotationVector.k;
        LOG_I(LOG_MODULE_FLIGHT, "Initial orientation acquired");
        break;
      }
    }
    delay(10);
  }
#else
  // LSM6DSO32 : Madgwick
  LOG_I(LOG_MODULE_FLIGHT, "Acquiring orientation from LSM6DSO32...");
  lsm6dso32_data_t imu_data;
  for (int i = 0; i < 20; i++) {
    if (LSM6DSO32_read(&lsm6dso32_dev, &imu_data)) {
      madgwick_update(&madgwick,
                      imu_data.gyro_x - gyro_offset_x,
                      imu_data.gyro_y - gyro_offset_y,
                      imu_data.gyro_z - gyro_offset_z,
                      imu_data.accel_x,
                      imu_data.accel_y,
                      imu_data.accel_z);
    }
    delay(10);
  }
  LOG_I(LOG_MODULE_FLIGHT, "Initial orientation acquired");
#endif

  // Init Kalman
  bmp5_data_t bmp_data;
  if (BMP5_read(&bmp5_dev, &bmp_data, current_qnh)) {
    kalman_config_t kalman_cfg = {
      .Q_altitude = KALMAN_PROCESS_NOISE_ALT,
      .Q_vario = KALMAN_PROCESS_NOISE_VARIO,
      .R_baro = KALMAN_MEASURE_NOISE_BARO,
      .R_imu = KALMAN_MEASURE_NOISE_IMU,
      .max_innovation_baro = 50.0f,
      .max_innovation_imu = 10.0f
    };
    kalman_init(&kalman, &kalman_cfg, bmp_data.altitude, 0.0f);
    LOG_I(LOG_MODULE_FLIGHT, "Kalman initialized at %.1fm", bmp_data.altitude);
  }

#if IMU_BNO08XX == 1
  // Attendre accélération linéaire
  wait_start = millis();
  while (millis() - wait_start < 1000) {
    if (bno08x_dev->getSensorEvent(&event)) {
      if (event.sensorId == SH2_LINEAR_ACCELERATION) {
        last_imu_data.accel_x = event.un.linearAcceleration.x;
        last_imu_data.accel_y = event.un.linearAcceleration.y;
        last_imu_data.accel_z = event.un.linearAcceleration.z;
        LOG_I(LOG_MODULE_FLIGHT, "Initial acceleration acquired");
        break;
      }
    }
    delay(10);
  }
#else
  if (LSM6DSO32_read(&lsm6dso32_dev, &imu_data)) {
    last_imu_data.accel_x = imu_data.accel_x;
    last_imu_data.accel_y = imu_data.accel_y;
    last_imu_data.accel_z = imu_data.accel_z;
  }
#endif*/

  // Créer tâche FreeRTOS
  task_running = true;
  BaseType_t ret = xTaskCreatePinnedToCore(
    task_flight_loop,
    "task_flight",
    TASK_FLIGHT_STACK_SIZE,
    NULL,
    TASK_FLIGHT_PRIORITY,
    &task_flight_handle,
    TASK_FLIGHT_CORE);

  if (ret != pdPASS) {
    LOG_E(LOG_MODULE_FLIGHT, "Task creation failed");
    task_running = false;
    vSemaphoreDelete(flight_data_mutex);
    return false;
  }

  LOG_I(LOG_MODULE_FLIGHT, "Task started on core %d", TASK_FLIGHT_CORE);
  return true;
}


void task_flight_stop() {
  task_running = false;
}

bool task_flight_is_running() {
  return task_running;
}

bool task_flight_get_data(flight_data_t* data) {
  if (!data) return false;

  if (flight_data_mutex == NULL) return false;

  if (xSemaphoreTake(flight_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    memcpy(data, &g_flight_data, sizeof(flight_data_t));
    xSemaphoreGive(flight_data_mutex);
    return true;
  }

  return false;
}

void task_flight_reset_kalman(float altitude) {
  kalman_reset(&kalman, altitude, 0.0f);
  LOG_I(LOG_MODULE_FLIGHT, "Kalman reset to %.1fm", altitude);
}

void task_flight_set_qnh(float qnh) {
  current_qnh = qnh;
  LOG_I(LOG_MODULE_FLIGHT, "QNH set to %.2f hPa", qnh);
}

#endif  // TASK_FLIGHT_H