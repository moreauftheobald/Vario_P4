/**
 * @file sensor_init.h
 * @brief Initialisation I2C capteurs + tâche FreeRTOS + calculs de vol
 * 
 * Bibliothèques requises:
 * - Adafruit BMP5xx
 * - Adafruit GPS Library
 * - Adafruit MAX1704X
 * - Adafruit BNO08x (si USE_BNO085)
 * - Adafruit LSM6DSOX + Adafruit AHRS (si !USE_BNO085)
 */

#ifndef SENSOR_INIT_H
#define SENSOR_INIT_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MAX1704X.h>
#include "src/config/config.h"
#include "src/config/pins.h"
#include "src/system/logger.h"
#include "src/system/kalman_vario.h"
#include "src/system/flight_data.h"
#include "src/system/flight_calc.h"

// IMU selon configuration
#ifdef USE_BNO085
#include <Adafruit_BNO08x.h>
#else
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_AHRS.h>
#endif

// =============================================================================
// VARIABLES GLOBALES
// =============================================================================
static TwoWire I2C_Sensors = TwoWire(1);
static TaskHandle_t task_sensors_handle = NULL;
static SemaphoreHandle_t i2c_mutex = NULL;

// Capteurs
static Adafruit_BMP5xx bmp;
static Adafruit_GPS gps(&I2C_Sensors);
static Adafruit_MAX17048 battery;

// Filtre de Kalman
static KalmanVario kalman;
static bool kalman_initialized = false;

// IMU
#ifdef USE_BNO085
static Adafruit_BNO08x imu(-1);
static sh2_SensorValue_t sensor_value;
static float quat_i = 0, quat_j = 0, quat_k = 0, quat_r = 0;
static float acc_x = 0, acc_y = 0, acc_z = 0;
static float gyro_x = 0, gyro_y = 0, gyro_z = 0;
#else
static Adafruit_LSM6DSOX lsm6;
static Adafruit_Madgwick filter;
static float quat_w = 1, quat_x = 0, quat_y = 0, quat_z = 0;
static float roll = 0, pitch = 0, yaw = 0;
static float acc_x = 0, acc_y = 0, acc_z = 0;
static float gyro_x = 0, gyro_y = 0, gyro_z = 0;
#endif

// Variables thermique
static float thermal_entry_alt = 0;
static uint32_t thermal_entry_time = 0;
static float thermal_vario_sum = 0;
static uint32_t thermal_vario_count = 0;

// Variables pour distance totale
static float last_lat = NAN, last_lon = NAN;

// =============================================================================
// FONCTIONS DE CALCUL AVANCÉES
// =============================================================================

/**
 * @brief Calcule la vitesse air estimée (TAS approximation)
 */
float calc_speed_air(float speed_gps, float wind_speed, float heading, float wind_dir) {
  // Calcul simplifié: correction par composante vent
  float heading_rad = heading * DEG_TO_RAD;
  float wind_rad = wind_dir * DEG_TO_RAD;

  // Composante vent dans direction de vol
  float wind_component = wind_speed * cosf(wind_rad - heading_rad);

  return speed_gps - wind_component;
}

/**
 * @brief Met à jour les statistiques max/min
 */
void update_statistics(FlightData* fd) {
  // Altitude
  if (fd->altitude_qne > fd->altitude_max) fd->altitude_max = fd->altitude_qne;
  if (fd->altitude_qne < fd->altitude_min || fd->altitude_min == 0) {
    fd->altitude_min = fd->altitude_qne;
  }

  // Vario
  if (fd->vario > fd->vario_max) fd->vario_max = fd->vario;
  if (fd->vario < fd->vario_min) fd->vario_min = fd->vario;

  // Vitesse
  if (fd->speed_gps > fd->speed_max) fd->speed_max = fd->speed_gps;

  // Finesse
  if (fd->glide_ratio > 0 && fd->glide_ratio < 50) {
    if (fd->glide_ratio > fd->glide_ratio_best) {
      fd->glide_ratio_best = fd->glide_ratio;
    }
  }

  // G-Force
  if (fd->g_force > fd->g_force_max) fd->g_force_max = fd->g_force;
  if (fd->g_force < fd->g_force_min) fd->g_force_min = fd->g_force;
}

/**
 * @brief Met à jour les données thermique
 */
void update_thermal_data(FlightData* fd, float vario, float altitude) {
  bool was_in_thermal = fd->in_thermal;
  fd->in_thermal = detect_thermal(vario, fd->in_thermal);

  if (fd->in_thermal) {
    // Entrée en thermique
    if (!was_in_thermal) {
      thermal_entry_alt = altitude;
      thermal_entry_time = millis();
      thermal_vario_sum = 0;
      thermal_vario_count = 0;
      LOG_V(LOG_FLIGHT, "Thermal entry at %.1fm", altitude);
    }

    // Calcul gain et moyenne
    fd->thermal_gain = altitude - thermal_entry_alt;
    fd->thermal_time = (millis() - thermal_entry_time) / 1000.0f;
    thermal_vario_sum += vario;
    thermal_vario_count++;
    fd->thermal_avg_vario = thermal_vario_sum / thermal_vario_count;

  } else if (was_in_thermal) {
    // Sortie de thermique
    LOG_V(LOG_FLIGHT, "Thermal exit: gain=%.1fm time=%.0fs avg=%.2fm/s",
          fd->thermal_gain, fd->thermal_time, fd->thermal_avg_vario);

    // Reset
    fd->thermal_gain = 0;
    fd->thermal_time = 0;
    fd->thermal_avg_vario = 0;
  }
}

/**
 * @brief Met à jour la distance cumulée
 */
void update_cumulative_distance(FlightData* fd) {
  if (!isnan(last_lat) && !isnan(last_lon) && fd->gps_fix) {
    float dist = calc_distance_gps(last_lat, last_lon,
                                   fd->latitude, fd->longitude);
    fd->distance_total += dist / 1000.0f;  // m -> km
  }

  last_lat = fd->latitude;
  last_lon = fd->longitude;
}

#ifdef USE_BNO085
static bool bno085_enable_report(sh2_SensorId_t sensor_id, uint32_t interval_us) {
  if (!imu.enableReport(sensor_id, interval_us)) {
    LOG_E(LOG_IMU, "Failed to enable report %d", sensor_id);
    return false;
  }
  return true;
}
#endif

// =============================================================================
// TÂCHE CAPTEURS
// =============================================================================

// =============================================================================
// TÂCHE CAPTEURS
// =============================================================================

void task_sensors(void* pvParameters) {
  LOG_I(LOG_SYSTEM, "Sensors task started on core %d", xPortGetCoreID());

  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t freq = pdMS_TO_TICKS(20);  // 50Hz
  uint32_t counter = 0;
  static char last_nmea[120] = "";

  // Variables capteurs
  float alt_baro = 0;
  float alt_gps = NAN;
  float acc_vertical = 0;
  float last_voltage = 0;
  float last_percent = 0;

  // Stabilisation
  const uint32_t STABILIZATION_CYCLES = 150;
  bool sensors_stabilized = false;

  // Variables pour estimation vent
  const uint32_t WIND_CALC_INTERVAL = 500;  // 10s à 50Hz

  // === MONITORING PERFORMANCE ===
  unsigned long cycle_time_us = 0;
  unsigned long cycle_time_max_us = 0;
  unsigned long cycle_time_sum_us = 0;
  uint32_t cycle_count = 0;
  const uint32_t MONITOR_INTERVAL = 500;  // Log toutes les 10s (500 cycles à 50Hz)

  while (true) {
    unsigned long cycle_start = millis();
    unsigned long cycle_start_us = micros();  // Timing précis
    unsigned long cycle_time_slot = cycle_start % 20;

    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (bmp.performReading()) {
        alt_baro = bmp.readAltitude(1013.25);
      }
      xSemaphoreGive(i2c_mutex);
    }

#ifdef USE_BNO085
    if (imu.wasReset()) {
      LOG_W(LOG_IMU, "BNO085 reset! Reconfiguring...");
      if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        bno085_enable_report(SH2_ROTATION_VECTOR, 20000);
        bno085_enable_report(SH2_LINEAR_ACCELERATION, 20000);
        bno085_enable_report(SH2_GYROSCOPE_CALIBRATED, 20000);
        xSemaphoreGive(i2c_mutex);
      }
    }

    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (imu.getSensorEvent(&sensor_value)) {
        switch (sensor_value.sensorId) {
          case SH2_ROTATION_VECTOR:
            quat_i = sensor_value.un.rotationVector.i;
            quat_j = sensor_value.un.rotationVector.j;
            quat_k = sensor_value.un.rotationVector.k;
            quat_r = sensor_value.un.rotationVector.real;
            break;
          case SH2_LINEAR_ACCELERATION:
            acc_x = sensor_value.un.linearAcceleration.x;
            acc_y = sensor_value.un.linearAcceleration.y;
            acc_z = sensor_value.un.linearAcceleration.z;
            break;
          case SH2_GYROSCOPE_CALIBRATED:
            gyro_x = sensor_value.un.gyroscope.x;
            gyro_y = sensor_value.un.gyroscope.y;
            gyro_z = sensor_value.un.gyroscope.z;
            break;
        }
        acc_vertical = kalman_get_vertical_accel(quat_r, quat_i, quat_j, quat_k,
                                                 acc_x, acc_y, acc_z, false);
      }
      xSemaphoreGive(i2c_mutex);
    }
#else
    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      sensors_event_t accel, gyro, temp;
      lsm6.getEvent(&accel, &gyro, &temp);
      acc_x = accel.acceleration.x;
      acc_y = accel.acceleration.y;
      acc_z = accel.acceleration.z;
      gyro_x = gyro.gyro.x;
      gyro_y = gyro.gyro.y;
      gyro_z = gyro.gyro.z;
      xSemaphoreGive(i2c_mutex);
    }

    filter.update(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, 0, 0, 0);
    filter.getQuaternion(&quat_w, &quat_x, &quat_y, &quat_z);
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();
    acc_vertical = kalman_get_vertical_accel(quat_w, quat_x, quat_y, quat_z,
                                             acc_x, acc_y, acc_z, true);
#endif
    for (int i = 0; i < 200; i++) {
      if (!gps.read()) break;
    }

    if (gps.newNMEAreceived()) {
      const char* nmea = gps.lastNMEA();
      strncpy(last_nmea, nmea, sizeof(last_nmea) - 1);
      last_nmea[sizeof(last_nmea) - 1] = '\0';
      gps.parse((char*)nmea);

      if (gps.fix) {
        alt_gps = gps.altitude;
      } else {
        alt_gps = NAN;
      }
    }

    if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      last_voltage = battery.cellVoltage();
      last_percent = battery.cellPercent();
      xSemaphoreGive(i2c_mutex);
    }
    // === STABILISATION ===
    if (counter == STABILIZATION_CYCLES) {
      sensors_stabilized = true;
      LOG_V(LOG_SYSTEM, "Sensors stabilized, initializing Kalman...");
    }

    // === INITIALISATION KALMAN ===
    if (!kalman_initialized && sensors_stabilized) {
      kalman_init(&kalman, alt_baro);
      kalman_initialized = true;
    }

    // === FUSION KALMAN + MISE À JOUR FLIGHT DATA ===
    if (kalman_initialized) {
      kalman_process(&kalman, acc_vertical, alt_baro, alt_gps);

      if (flight_data_lock(10)) {
        // === ALTITUDES ===
        flight_data.altitude_qne = kalman.altitude;
        flight_data.altitude_qnh = calc_altitude_qnh(bmp.pressure, flight_data.qnh_local);
        flight_data.altitude_gps = alt_gps;

        if (flight_data.flight_started) {
          flight_data.altitude_qfe = calc_altitude_qfe(kalman.altitude,
                                                       flight_data.altitude_takeoff);
        }

        // === VARIO ===
        flight_data.vario = kalman.vario;
        flight_data.vario_integrated = calc_vario_integrated(kalman.vario);
        flight_data.speed_vertical = kalman.vario;

        // === GPS ===
        if (gps.fix) {
          flight_data.gps_fix = true;
          flight_data.latitude = gps.latitudeDegrees;
          flight_data.longitude = gps.longitudeDegrees;
          flight_data.speed_gps = gps.speed * 1.852f;  // Noeuds -> km/h
          flight_data.heading = gps.angle;
          flight_data.satellites = gps.satellites;

          // === VOL DÉMARRÉ ===
          if (flight_data.flight_started) {
            // Distance depuis décollage
            flight_data.distance_takeoff = calc_distance_gps(
                                             flight_data.latitude_takeoff, flight_data.longitude_takeoff,
                                             flight_data.latitude, flight_data.longitude)
                                           / 1000.0f;

            // Distance linéaire
            if (flight_data.distance_takeoff > flight_data.distance_linear) {
              flight_data.distance_linear = flight_data.distance_takeoff;
            }

            // Distance cumulée
            update_cumulative_distance(&flight_data);

            // Cap vers décollage
            flight_data.heading_takeoff = calc_bearing_gps(
              flight_data.latitude, flight_data.longitude,
              flight_data.latitude_takeoff, flight_data.longitude_takeoff);

            // Finesse
            if (fabsf(kalman.vario) > 0.3f) {
              float speed_ms = gps.speed * 0.51444f;
              if (speed_ms > 5.0f) {
                flight_data.glide_ratio = calc_glide_ratio(speed_ms, kalman.vario);
                flight_data.glide_ratio_avg = calc_glide_ratio_avg(flight_data.glide_ratio);
              }
            }

            // Vent
            if (counter % WIND_CALC_INTERVAL == 0 && flight_data.speed_gps > 10.0f) {
              float speed_air_ms = flight_data.speed_gps / 3.6f;
              calc_wind(gps.speed * 0.51444f, gps.angle, speed_air_ms,
                        &flight_data.wind_speed, &flight_data.wind_direction);
              flight_data.wind_speed *= 3.6f;

              flight_data.speed_air = calc_speed_air(flight_data.speed_gps,
                                                     flight_data.wind_speed,
                                                     flight_data.heading,
                                                     flight_data.wind_direction);
            }

            // Temps de vol
            flight_data.flight_time = (millis() - flight_data.flight_start_time) / 1000;

            // Thermique
            update_thermal_data(&flight_data, kalman.vario, kalman.altitude);
          }

        } else {
          flight_data.gps_fix = false;
        }

        // === PRESSION & TEMPÉRATURE ===
        flight_data.pressure = bmp.pressure;
        flight_data.temperature = bmp.temperature;

        // === BATTERIE ===
        flight_data.battery_voltage = last_voltage;
        flight_data.battery_percent = last_percent;

        // === G-FORCE ===
        flight_data.g_force = calc_g_force(acc_x, acc_y, acc_z);

        // === DÉTECTION DÉBUT VOL ===
        if (!flight_data.flight_started) {
          flight_data.flight_started = detect_flight_start(
            kalman.vario, flight_data.speed_gps,
            kalman.altitude, false);

          if (flight_data.flight_started) {
            flight_data.altitude_takeoff = kalman.altitude;
            flight_data.latitude_takeoff = flight_data.latitude;
            flight_data.longitude_takeoff = flight_data.longitude;
            flight_data.flight_start_time = millis();
            flight_data.altitude_min = kalman.altitude;
            flight_data.altitude_max = kalman.altitude;
            last_lat = flight_data.latitude;
            last_lon = flight_data.longitude;
            LOG_V(LOG_FLIGHT, "Flight started at %.1fm!", kalman.altitude);
          }
        }

        // === STATISTIQUES ===
        if (flight_data.flight_started) {
          update_statistics(&flight_data);
        }

        // === TIMESTAMP ===
        flight_data.last_update_ms = millis();

        flight_data_unlock();
      }
    }

    // === MESURE TEMPS D'EXÉCUTION ===
    cycle_time_us = micros() - cycle_start_us;
    cycle_time_sum_us += cycle_time_us;
    cycle_count++;
    if (cycle_time_us > cycle_time_max_us) {
      cycle_time_max_us = cycle_time_us;
    }

    // === LOGS CAPTEURS 1Hz ===
    if (counter % 50 == 0) {
      if (last_nmea[0] != '\0') {
        LOG_V(LOG_GPS, "%s", last_nmea);
      }

      if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (bmp.performReading()) {
          LOG_V(LOG_BARO, "T=%.1f°C P=%.1fhPa Alt=%.1fm",
                bmp.temperature, bmp.pressure, alt_baro);
        }
        xSemaphoreGive(i2c_mutex);
      }

      LOG_V(LOG_BATTERY, "Battery: %.2fV (%.1f%%)", last_voltage, last_percent);

#ifdef USE_BNO085
      LOG_V(LOG_IMU, "Q: i=%.3f j=%.3f k=%.3f r=%.3f Acc: x=%.2f y=%.2f z=%.2f",
            quat_i, quat_j, quat_k, quat_r, acc_x, acc_y, acc_z);
#else
      LOG_V(LOG_IMU, "Q: w=%.3f x=%.3f y=%.3f z=%.3f RPY: %.1f° %.1f° %.1f°",
            quat_w, quat_x, quat_y, quat_z, roll, pitch, yaw);
#endif

      if (kalman_initialized) {
        LOG_I(LOG_KALMAN, ">>> Alt=%.1fm Vario=%.2fm/s AccZ=%.2f",
              kalman.altitude, kalman.vario, acc_vertical);
      }
    }

    // === MONITORING PERFORMANCE (toutes les 10s) ===
    if (counter % MONITOR_INTERVAL == 0 && counter > 0) {
      // Calcul moyenne et CPU%
      unsigned long cycle_time_avg_us = cycle_time_sum_us / cycle_count;
      float cpu_usage = (cycle_time_avg_us / 20000.0f) * 100.0f;  // % du cycle 20ms

      // Mémoire stack (high water mark = minimum restant)
      UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
      size_t stack_used = SENSORS_TASK_STACK_SIZE - (stack_hwm * sizeof(StackType_t));

      // Mémoire heap
      size_t free_sram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
      size_t total_sram = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
      size_t used_sram = total_sram - free_sram;

      size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
      size_t total_psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
      size_t used_psram = total_psram - free_psram;

      LOG_V(LOG_SYSTEM, "╔═══════════════════════════════════════════════╗");
      LOG_V(LOG_SYSTEM, "║       SENSORS TASK PERFORMANCE MONITOR       ║");
      LOG_V(LOG_SYSTEM, "╠═══════════════════════════════════════════════╣");
      LOG_I(LOG_SYSTEM, "║ Cycle Time:                                   ║");
      LOG_I(LOG_SYSTEM, "║   Average: %6lu µs  (%.1f%% CPU)           ║",
            cycle_time_avg_us, cpu_usage);
      LOG_I(LOG_SYSTEM, "║   Maximum: %6lu µs                          ║",
            cycle_time_max_us);
      LOG_I(LOG_SYSTEM, "║   Budget:   20000 µs  (50Hz)                 ║");
      LOG_V(LOG_SYSTEM, "╠═══════════════════════════════════════════════╣");
      LOG_V(LOG_SYSTEM, "║ Stack Usage:                                  ║");
      LOG_V(LOG_SYSTEM, "║   Used:     %5zu / %d bytes (%3d%%)        ║",
            stack_used, SENSORS_TASK_STACK_SIZE,  // ← Utiliser la constante
            (int)((stack_used * 100) / SENSORS_TASK_STACK_SIZE));
      LOG_V(LOG_SYSTEM, "║   Free:     %5zu bytes (HWM: %u)             ║",
            stack_hwm * sizeof(StackType_t), stack_hwm);
      LOG_V(LOG_SYSTEM, "╠═══════════════════════════════════════════════╣");
      LOG_V(LOG_SYSTEM, "║ Heap Memory:                                  ║");
      LOG_V(LOG_SYSTEM, "║   SRAM:  %7zu / %7zu KB (%3d%%)          ║",
            used_sram / 1024, total_sram / 1024,
            (int)((used_sram * 100) / total_sram));
      LOG_V(LOG_SYSTEM, "║   PSRAM: %7zu / %7zu KB (%3d%%)          ║",
            used_psram / 1024, total_psram / 1024,
            (int)((used_psram * 100) / total_psram));
      LOG_V(LOG_SYSTEM, "╚═══════════════════════════════════════════════╝");

      // Reset stats pour prochaine période
      cycle_time_sum_us = 0;
      cycle_count = 0;
      cycle_time_max_us = 0;
    }

    counter++;
    vTaskDelayUntil(&last_wake, freq);
  }
}

// =============================================================================
// FONCTIONS D'INITIALISATION
// =============================================================================

bool sensors_init_i2c() {
  LOG_I(LOG_SYSTEM, "Initializing I2C sensors...");

  i2c_mutex = xSemaphoreCreateMutex();
  if (i2c_mutex == NULL) {
    LOG_E(LOG_SYSTEM, "Failed to create I2C mutex");
    return false;
  }

  if (!I2C_Sensors.begin(I2C_SENSORS_SDA, I2C_SENSORS_SCL, I2C_SENSORS_FREQ)) {
    LOG_E(LOG_SYSTEM, "I2C init failed");
    return false;
  }

  LOG_I(LOG_SYSTEM, "I2C OK (SDA=%d, SCL=%d, %dkHz)",
        I2C_SENSORS_SDA, I2C_SENSORS_SCL, I2C_SENSORS_FREQ / 1000);
  return true;
}

bool sensors_init_bmp585() {
  LOG_I(LOG_BARO, "Initializing BMP585...");

  if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_BARO, "Failed to take I2C mutex");
    return false;
  }

  if (!bmp.begin(BMP5_I2C_ADDR_PRIM, &I2C_Sensors)) {
    if (!bmp.begin(BMP5_I2C_ADDR_SEC, &I2C_Sensors)) {
      LOG_E(LOG_BARO, "BMP585 not found");
      xSemaphoreGive(i2c_mutex);
      return false;
    }
  }

  bmp.setTemperatureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_8X);
  bmp.setPressureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff((bmp5xx_iir_filter_t)BMP5_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate((bmp5xx_odr_t)BMP5_ODR_50_HZ);

  xSemaphoreGive(i2c_mutex);

  LOG_I(LOG_BARO, "BMP585 OK (50Hz)");
  return true;
}

bool sensors_init_gps() {
  LOG_I(LOG_GPS, "Initializing GPS PA1010D...");

  if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_GPS, "Failed to take I2C mutex");
    return false;
  }

  if (!gps.begin(0x10)) {
    if (!gps.begin(0x42)) {
      LOG_E(LOG_GPS, "GPS not found");
      xSemaphoreGive(i2c_mutex);
      return false;
    }
  }

  xSemaphoreGive(i2c_mutex);
  delay(100);

  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  delay(100);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  delay(100);
  gps.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  delay(100);

  LOG_I(LOG_GPS, "GPS PA1010D OK (5Hz, RMC+GGA)");
  return true;
}

bool sensors_init_battery() {
  LOG_I(LOG_BATTERY, "Initializing MAX17048...");

  if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_BATTERY, "Failed to take I2C mutex");
    return false;
  }

  if (!battery.begin(&I2C_Sensors)) {
    LOG_E(LOG_BATTERY, "MAX17048 not found");
    xSemaphoreGive(i2c_mutex);
    return false;
  }

  float voltage = battery.cellVoltage();
  float percent = battery.cellPercent();

  xSemaphoreGive(i2c_mutex);

  LOG_I(LOG_BATTERY, "MAX17048 OK (%.2fV, %.1f%%)", voltage, percent);
  return true;
}

bool sensors_init_imu() {
#ifdef USE_BNO085
  LOG_I(LOG_IMU, "Initializing BNO085...");

  if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_IMU, "Failed to take I2C mutex");
    return false;
  }

  if (!imu.begin_I2C(BNO08x_I2CADDR_DEFAULT, &I2C_Sensors)) {
    if (!imu.begin_I2C(0x4B, &I2C_Sensors)) {
      LOG_E(LOG_IMU, "BNO085 not found");
      xSemaphoreGive(i2c_mutex);
      return false;
    }
  }

  LOG_D(LOG_IMU, "BNO085 found, configuring reports...");

  if (!bno085_enable_report(SH2_ROTATION_VECTOR, 20000)) {
    xSemaphoreGive(i2c_mutex);
    return false;
  }
  if (!bno085_enable_report(SH2_LINEAR_ACCELERATION, 20000)) {
    xSemaphoreGive(i2c_mutex);
    return false;
  }
  if (!bno085_enable_report(SH2_GYROSCOPE_CALIBRATED, 20000)) {
    xSemaphoreGive(i2c_mutex);
    return false;
  }

  xSemaphoreGive(i2c_mutex);

  LOG_I(LOG_IMU, "BNO085 OK (50Hz, Quat+Acc+Gyro)");
  return true;

#else
  LOG_I(LOG_IMU, "Initializing LSM6DSO32...");

  if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_IMU, "Failed to take I2C mutex");
    return false;
  }

  if (!lsm6.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &I2C_Sensors)) {
    if (!lsm6.begin_I2C(0x6B, &I2C_Sensors)) {
      LOG_E(LOG_IMU, "LSM6DSO32 not found");
      xSemaphoreGive(i2c_mutex);
      return false;
    }
  }

  lsm6.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  lsm6.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6.setGyroDataRate(LSM6DS_RATE_104_HZ);

  xSemaphoreGive(i2c_mutex);

  filter.begin(50);

  LOG_I(LOG_IMU, "LSM6DSO32 OK (104Hz, 16G, 1000dps, AHRS 50Hz)");
  return true;
#endif
}

bool sensors_create_task(BaseType_t core) {
  BaseType_t result = xTaskCreatePinnedToCore(
    task_sensors,
    "Sensors",
    SENSORS_TASK_STACK_SIZE,  // Stack augmenté pour calculs supplémentaires
    NULL,
    2,
    &task_sensors_handle,
    core);

  if (result != pdPASS) {
    LOG_E(LOG_SYSTEM, "Sensors task creation failed");
    return false;
  }

  LOG_V(LOG_SYSTEM, "Sensors task created on core %d", core);
  return true;
}

bool init_sensors_global(BaseType_t core) {
  if (!flight_data_init()) {
    LOG_E(LOG_SYSTEM, "Failed to init flight data");
    return false;
  }

  if (!sensors_init_i2c()) return false;

  if (!sensors_init_bmp585()) {
    LOG_W(LOG_BARO, "BMP585 init failed, continuing...");
  }

  if (!sensors_init_gps()) {
    LOG_W(LOG_GPS, "GPS init failed, continuing...");
  }

  if (!sensors_init_battery()) {
    LOG_W(LOG_BATTERY, "MAX17048 init failed, continuing...");
  }

  if (!sensors_init_imu()) {
    LOG_W(LOG_IMU, "IMU init failed, continuing...");
  }

  if (!sensors_create_task(core)) {
    return false;
  }

  return true;
}

#endif  // SENSOR_INIT_H