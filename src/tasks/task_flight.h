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
// MONITORING
// =============================================================================
#define TASK_FLIGHT_MONITOR_ENABLED 1
#define MONITOR_PRINT_INTERVAL_MS 10000

typedef struct {
  uint32_t cycle_time_us;
  uint32_t cycle_time_max_us;
  uint32_t cycle_time_min_us;
  uint32_t cycle_time_avg_us;
  
  uint32_t imu_time_us;
  uint32_t bmp5_time_us;
  uint32_t kalman_time_us;
  uint32_t gps_time_us;
  uint32_t calculs_time_us;
  
  uint32_t overrun_count;
  uint32_t overrun_max_us;
  uint32_t total_cycles;
  uint64_t total_time_us;
  uint32_t last_print_ms;
} task_flight_monitor_t;

static task_flight_monitor_t monitor = { 0 };

// =============================================================================
// CONFIGURATION
// =============================================================================
#define TASK_SENSORS_FREQ_HZ 200
#define TASK_SENSORS_PERIOD_MS 5
#define TASK_FLIGHT_STACK_SIZE 8192
#define TASK_FLIGHT_PRIORITY 5
#define TASK_FLIGHT_CORE 1

#define FREQ_GPS_DIVIDER 100      // 2 Hz
#define FREQ_FLIGHT_CALCULS_DIVIDER 100  // 2 Hz

#define TAKEOFF_SPEED_THRESHOLD 10.0f
#define TAKEOFF_VARIO_THRESHOLD 0.5f
#define LANDING_SPEED_THRESHOLD 3.0f
#define LANDING_TIME_MS 5000
#define SENSOR_TIMEOUT_MS 2000

// =============================================================================
// VARIABLES PRIVÉES
// =============================================================================
typedef struct {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
} imu_data_cache_t;

static TaskHandle_t task_flight_handle = NULL;
static SemaphoreHandle_t flight_data_mutex = NULL;
static madgwick_filter_t madgwick;
static kalman_filter_t kalman;
static bool task_running = false;

static imu_data_cache_t last_imu_data;
static bmp5_data_t last_bmp_data;
static max17048_data_t last_battery_data;

static uint32_t last_bmp_read_time = 0;
static uint32_t last_battery_read_time = 0;
static uint32_t last_baro_update = 0;
static uint32_t last_imu_update = 0;

static float current_qnh = 1013.25f;
static float gyro_offset_x = -0.001119f;
static float gyro_offset_y = -0.001825f;
static float gyro_offset_z = -0.000743f;

// =============================================================================
// MONITORING FUNCTIONS
// =============================================================================
#if TASK_FLIGHT_MONITOR_ENABLED

static void monitor_init() {
  monitor.cycle_time_min_us = UINT32_MAX;
  monitor.cycle_time_max_us = 0;
  monitor.total_cycles = 0;
  monitor.total_time_us = 0;
  monitor.overrun_count = 0;
  monitor.overrun_max_us = 0;
  monitor.last_print_ms = millis();
}

static uint32_t monitor_cycle_start() {
  return micros();
}

static void monitor_cycle_end(uint32_t start_us) {
  uint32_t elapsed = micros() - start_us;
  
  monitor.cycle_time_us = elapsed;
  monitor.total_time_us += elapsed;
  monitor.total_cycles++;
  
  if (elapsed > monitor.cycle_time_max_us) {
    monitor.cycle_time_max_us = elapsed;
  }
  if (elapsed < monitor.cycle_time_min_us) {
    monitor.cycle_time_min_us = elapsed;
  }
  
  monitor.cycle_time_avg_us = monitor.total_time_us / monitor.total_cycles;
  
  uint32_t budget_us = TASK_SENSORS_PERIOD_MS * 1000;
  if (elapsed > budget_us) {
    monitor.overrun_count++;
    if (elapsed > monitor.overrun_max_us) {
      monitor.overrun_max_us = elapsed;
    }
  }
}

static uint32_t monitor_func_time(uint32_t start_us) {
  return micros() - start_us;
}

static void monitor_print_stats() {
  uint32_t now = millis();
  
  if (now - monitor.last_print_ms < MONITOR_PRINT_INTERVAL_MS) {
    return;
  }
  
  monitor.last_print_ms = now;
  
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
  
  LOG_I(LOG_MODULE_FLIGHT, "CPU usage: %.1f%%", cpu_usage);
  
  if (monitor.overrun_count > 0) {
    LOG_W(LOG_MODULE_FLIGHT, "Worst overrun: %lu µs", monitor.overrun_max_us);
  }
  
  LOG_I(LOG_MODULE_FLIGHT, "Function times (last):");
  LOG_I(LOG_MODULE_FLIGHT, "  IMU: %lu µs", monitor.imu_time_us);
  LOG_I(LOG_MODULE_FLIGHT, "  BMP5: %lu µs", monitor.bmp5_time_us);
  LOG_I(LOG_MODULE_FLIGHT, "  Kalman: %lu µs", monitor.kalman_time_us);
  LOG_I(LOG_MODULE_FLIGHT, "  GPS: %lu µs", monitor.gps_time_us);
  LOG_I(LOG_MODULE_FLIGHT, "  Calculs: %lu µs", monitor.calculs_time_us);
  LOG_I(LOG_MODULE_FLIGHT, "================================");
}

#endif

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

/**
 * @brief Lecture IMU commune (BNO08x ou LSM6DSO32)
 */
static bool read_imu_data() {
  if (!sensor_imu_ready) return false;

#if IMU_BNO08XX == 1
  // BNO08x : polling événements
  if (bno08x_dev && bno08x_dev->dataAvailable()) {
    // TODO: extraire quaternions et accel linéaire
    last_imu_data.accel_x = 0;
    last_imu_data.accel_y = 0;
    last_imu_data.accel_z = 0;
    return true;
  }
  return false;

#else
  // LSM6DSO32 : lecture directe
  lsm6dso32_data_t imu_data;
  if (LSM6DSO32_read(&lsm6dso32_dev, &imu_data)) {
    last_imu_data.accel_x = imu_data.accel_x;
    last_imu_data.accel_y = imu_data.accel_y;
    last_imu_data.accel_z = imu_data.accel_z;
    last_imu_data.gyro_x = imu_data.gyro_x - gyro_offset_x;
    last_imu_data.gyro_y = imu_data.gyro_y - gyro_offset_y;
    last_imu_data.gyro_z = imu_data.gyro_z - gyro_offset_z;
    
    // Update Madgwick
    madgwick_update(&madgwick,
                    last_imu_data.gyro_x,
                    last_imu_data.gyro_y,
                    last_imu_data.gyro_z,
                    last_imu_data.accel_x,
                    last_imu_data.accel_y,
                    last_imu_data.accel_z);
    return true;
  }
  return false;
#endif
}

/**
 * @brief Lecture GPS
 */
static bool read_gps() {
  if (!sensor_gps_ready) return false;

  bool new_data = sensor_read_gps();

  if (new_data && gps_data.fix) {
    g_flight_data.gps_fix = gps_data.fix;
    g_flight_data.satellites = gps_data.satellites;
    g_flight_data.hdop = gps_data.hdop;
    g_flight_data.latitude = gps_data.latitude;
    g_flight_data.longitude = gps_data.longitude;
    g_flight_data.altitude_gps = gps_data.altitude;
    g_flight_data.speed_ground = gps_data.speed * 1.852f;
    g_flight_data.heading = gps_data.course;
    
    g_flight_data.hour = gps_data.hour;
    g_flight_data.minute = gps_data.minute;
    g_flight_data.second = gps_data.second;
    g_flight_data.day = gps_data.day;
    g_flight_data.month = gps_data.month;
    g_flight_data.year = gps_data.year;
    
    g_flight_data.sensors_health.gps_healthy = true;
    g_flight_data.sensors_health.last_gps_success = millis();
  }

  return new_data;
}

/**
 * @brief Update Kalman sélectif
 */
static void update_kalman_selective(bool update_baro) {
  float accel_vertical = madgwick_get_vertical_accel(&madgwick,
                                                     last_imu_data.accel_x,
                                                     last_imu_data.accel_y,
                                                     last_imu_data.accel_z);

  float dt = TASK_SENSORS_PERIOD_MS / 1000.0f;
  kalman_predict(&kalman, accel_vertical, dt);

  if (update_baro) {
    kalman_update_baro(&kalman, last_bmp_data.altitude);
  }

  g_flight_data.altitude_qnh = kalman_get_altitude(&kalman);
  g_flight_data.vario = kalman_get_vario(&kalman);
  g_flight_data.kalman_converged = kalman_is_converged(&kalman);
}

/**
 * @brief Prédiction Kalman seule
 */
static void kalman_predict_only() {
  float accel_vertical = madgwick_get_vertical_accel(&madgwick,
                                                     last_imu_data.accel_x,
                                                     last_imu_data.accel_y,
                                                     last_imu_data.accel_z);

  float dt = TASK_SENSORS_PERIOD_MS / 1000.0f;
  kalman_predict(&kalman, accel_vertical, dt);

  g_flight_data.altitude_qnh = kalman_get_altitude(&kalman);
  g_flight_data.vario = kalman_get_vario(&kalman);
}

/**
 * @brief Calcule paramètres de vol
 */
static void calculate_flight_params() {
  g_flight_data.altitude_qne = last_bmp_data.altitude;

  if (g_flight_data.in_flight) {
    g_flight_data.altitude_agl = g_flight_data.altitude_qnh - g_flight_data.altitude_takeoff;
  } else {
    g_flight_data.altitude_agl = 0.0f;
  }

  if (g_flight_data.altitude_qnh > g_flight_data.altitude_max) {
    g_flight_data.altitude_max = g_flight_data.altitude_qnh;
  }
  if (g_flight_data.altitude_qnh < g_flight_data.altitude_min) {
    g_flight_data.altitude_min = g_flight_data.altitude_qnh;
  }

  if (g_flight_data.vario > g_flight_data.vario_max) {
    g_flight_data.vario_max = g_flight_data.vario;
  }
  if (g_flight_data.vario < g_flight_data.vario_min) {
    g_flight_data.vario_min = g_flight_data.vario;
  }

  euler_t euler;
  madgwick_quaternion_to_euler(&madgwick.q, &euler);
  g_flight_data.roll = euler.roll;
  g_flight_data.pitch = euler.pitch;
  g_flight_data.yaw = euler.yaw;

  g_flight_data.quaternion.w = madgwick.q.w;
  g_flight_data.quaternion.x = madgwick.q.x;
  g_flight_data.quaternion.y = madgwick.q.y;
  g_flight_data.quaternion.z = madgwick.q.z;

  g_flight_data.pressure = last_bmp_data.pressure;
  g_flight_data.temperature = last_bmp_data.temperature;
  g_flight_data.qnh = current_qnh;

  g_flight_data.accel_vertical = madgwick_get_vertical_accel(&madgwick,
                                                             last_imu_data.accel_x,
                                                             last_imu_data.accel_y,
                                                             last_imu_data.accel_z);

  float g_total = sqrtf(last_imu_data.accel_x * last_imu_data.accel_x + 
                       last_imu_data.accel_y * last_imu_data.accel_y + 
                       last_imu_data.accel_z * last_imu_data.accel_z) / 9.80665f;
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
    if (g_flight_data.speed_ground > TAKEOFF_SPEED_THRESHOLD || 
        fabsf(g_flight_data.vario) > TAKEOFF_VARIO_THRESHOLD) {
      g_flight_data.in_flight = true;
      g_flight_data.takeoff_time = millis();
      g_flight_data.altitude_takeoff = g_flight_data.altitude_qnh;
      LOG_I(LOG_MODULE_FLIGHT, "Takeoff detected at %.1fm", g_flight_data.altitude_takeoff);
    }
  } else {
    if (g_flight_data.speed_ground < LANDING_SPEED_THRESHOLD && 
        fabsf(g_flight_data.vario) < 0.2f) {
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

    g_flight_data.time_flight = (millis() - g_flight_data.takeoff_time) / 1000;
  }

  uint8_t quality = 0;
  if (g_flight_data.sensors_health.imu_healthy) quality += 33;
  if (g_flight_data.sensors_health.baro_healthy) quality += 33;
  if (g_flight_data.sensors_health.gps_healthy) quality += 34;
  g_flight_data.data_quality = quality;

  g_flight_data.timestamp_ms = millis();
}

/**
 * @brief Vérifie santé capteurs
 */
static void check_sensors_health() {
  uint32_t now = millis();

  if (now - g_flight_data.sensors_health.last_imu_success > SENSOR_TIMEOUT_MS) {
    if (g_flight_data.sensors_health.imu_healthy) {
      g_flight_data.sensors_health.imu_healthy = false;
      LOG_E(LOG_MODULE_FLIGHT, "IMU timeout");
    }
  }

  if (now - g_flight_data.sensors_health.last_baro_success > SENSOR_TIMEOUT_MS) {
    if (g_flight_data.sensors_health.baro_healthy) {
      g_flight_data.sensors_health.baro_healthy = false;
      LOG_E(LOG_MODULE_FLIGHT, "BMP5 timeout");
    }
  }

  if (now - g_flight_data.sensors_health.last_gps_success > SENSOR_TIMEOUT_MS) {
    if (g_flight_data.sensors_health.gps_healthy) {
      g_flight_data.sensors_health.gps_healthy = false;
      LOG_W(LOG_MODULE_FLIGHT, "GPS timeout");
    }
  }
}

// =============================================================================
// TÂCHE FREERTOS
// =============================================================================

static void task_flight_loop(void* parameter) {
  TickType_t last_wake_time = xTaskGetTickCount();
  uint32_t cycle_count = 0;

  LOG_I(LOG_MODULE_FLIGHT, "Loop started @ %d Hz", TASK_SENSORS_FREQ_HZ);

#if TASK_FLIGHT_MONITOR_ENABLED
  monitor_init();
#endif

  while (task_running) {
#if TASK_FLIGHT_MONITOR_ENABLED
    uint32_t cycle_start = monitor_cycle_start();
    uint32_t func_start;
#endif

    uint32_t now = millis();

    // IMU (200 Hz)
#if TASK_FLIGHT_MONITOR_ENABLED
    func_start = micros();
#endif

    if (read_imu_data()) {
      g_flight_data.sensors_health.last_imu_success = now;
      g_flight_data.sensors_health.imu_healthy = true;
    }

#if TASK_FLIGHT_MONITOR_ENABLED
    monitor.imu_time_us = monitor_func_time(func_start);
#endif

    // BMP5 (100 Hz = 10ms)
    if (now - last_bmp_read_time >= 10) {
#if TASK_FLIGHT_MONITOR_ENABLED
      func_start = micros();
#endif

      if (BMP5_read(&bmp5_dev, &last_bmp_data, current_qnh)) {
        last_bmp_read_time = now;
        g_flight_data.sensors_health.last_baro_success = now;
        g_flight_data.sensors_health.baro_healthy = true;
      }

#if TASK_FLIGHT_MONITOR_ENABLED
      monitor.bmp5_time_us = monitor_func_time(func_start);
#endif
    }

    // Kalman
#if TASK_FLIGHT_MONITOR_ENABLED
    func_start = micros();
#endif

    bool imu_update_needed = (now - last_imu_update >= 20);   // 50 Hz
    bool baro_update_needed = (now - last_baro_update >= 200); // 5 Hz

    if (imu_update_needed || baro_update_needed) {
      update_kalman_selective(baro_update_needed);
      if (imu_update_needed) last_imu_update = now;
      if (baro_update_needed) last_baro_update = now;
    } else {
      kalman_predict_only();
    }

#if TASK_FLIGHT_MONITOR_ENABLED
    monitor.kalman_time_us = monitor_func_time(func_start);
#endif

    // GPS (2 Hz)
    if (cycle_count % FREQ_GPS_DIVIDER == 0) {
#if TASK_FLIGHT_MONITOR_ENABLED
      func_start = micros();
#endif

      read_gps();

#if TASK_FLIGHT_MONITOR_ENABLED
      monitor.gps_time_us = monitor_func_time(func_start);
#endif
    }

    // Batterie (1 Hz)
    if (now - last_battery_read_time >= 1000) {
      if (MAX17048_read(&max17048_dev, &last_battery_data)) {
        last_battery_read_time = now;
        g_flight_data.battery_voltage = last_battery_data.voltage;
        g_flight_data.battery_percent = (uint8_t)last_battery_data.soc;
      }
    }

    // Calculs (2 Hz)
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
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_SENSORS_PERIOD_MS));
  }

  LOG_I(LOG_MODULE_FLIGHT, "Task stopped");
  vTaskDelete(NULL);
}

// =============================================================================
// FONCTIONS PUBLIQUES
// =============================================================================

bool task_flight_start() {
  if (task_running) return true;

  LOG_I(LOG_MODULE_FLIGHT, "Starting task");

  flight_data_mutex = xSemaphoreCreateMutex();
  if (!flight_data_mutex) {
    LOG_E(LOG_MODULE_FLIGHT, "Mutex creation failed");
    return false;
  }

  madgwick_init(&madgwick, TASK_SENSORS_FREQ_HZ, 0.1f);

#if IMU_BNO08XX == 1
  LOG_I(LOG_MODULE_FLIGHT, "Using BNO08x hardware fusion");
#else
  LOG_I(LOG_MODULE_FLIGHT, "Using Madgwick filter");
  
  // Acquisition orientation LSM6DSO32
  for (int i = 0; i < 20; i++) {
    if (read_imu_data()) {
      delay(10);
    }
  }
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
    LOG_I(LOG_MODULE_FLIGHT, "Kalman init at %.1fm", bmp_data.altitude);
  }

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
  if (!data || !flight_data_mutex) return false;

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