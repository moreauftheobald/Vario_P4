/**
 * @file sensor_init.h
 * @brief Initialisation I2C capteurs + tâche FreeRTOS
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
static TwoWire I2C_Sensors = TwoWire(1);  // Wire1 pour capteurs
static TaskHandle_t task_sensors_handle = NULL;
static SemaphoreHandle_t i2c_mutex = NULL;  // Mutex pour protéger le bus I2C

// Capteurs
static Adafruit_BMP5xx bmp;
static Adafruit_GPS gps(&I2C_Sensors);
static Adafruit_MAX17048 battery;

// Filtre de Kalman
static KalmanVario kalman;
static bool kalman_initialized = false;

// IMU
#ifdef USE_BNO085
  static Adafruit_BNO08x imu(-1);  // Pas de reset pin
  static sh2_SensorValue_t sensor_value;
  
  // Dernières valeurs pour log (mises à jour en temps réel)
  static float quat_i = 0, quat_j = 0, quat_k = 0, quat_r = 0;
  static float acc_x = 0, acc_y = 0, acc_z = 0;
  static float gyro_x = 0, gyro_y = 0, gyro_z = 0;
#else
  static Adafruit_LSM6DSOX lsm6;
  static Adafruit_Madgwick filter;
  
  // Dernières valeurs pour log
  static float quat_w = 1, quat_x = 0, quat_y = 0, quat_z = 0;
  static float roll = 0, pitch = 0, yaw = 0;
  static float acc_x = 0, acc_y = 0, acc_z = 0;
  static float gyro_x = 0, gyro_y = 0, gyro_z = 0;
#endif

// =============================================================================
// TÂCHE CAPTEURS
// =============================================================================

/**
 * @brief Tâche FreeRTOS pour lecture capteurs
 * @param pvParameters Paramètres (non utilisé)
 */
void task_sensors(void* pvParameters) {
  LOG_I(LOG_SYSTEM, "Sensors task started on core %d", xPortGetCoreID());

  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t freq = pdMS_TO_TICKS(20);  // 50Hz (plus stable pour I2C)
  uint32_t counter = 0;
  static char last_nmea[120] = "";
  
  // Variables pour Kalman
  float alt_baro = 0;
  float alt_gps = NAN;
  float acc_vertical = 0;
  
  // Variables pour capteurs à basse fréquence
  float last_voltage = 0;
  float last_percent = 0;
  
  // Délai de stabilisation capteurs (3 secondes = 150 cycles à 50Hz)
  const uint32_t STABILIZATION_CYCLES = 150;
  bool sensors_stabilized = false;

  while (true) {
    unsigned long cycle_start = millis();
    unsigned long cycle_time = cycle_start % 20;  // Position dans le cycle de 20ms
    
    // === SLOT 0-5ms: BMP585 ===
    if (cycle_time < 5) {
      if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (bmp.performReading()) {
          alt_baro = bmp.readAltitude(1013.25);  // QNE
        }
        xSemaphoreGive(i2c_mutex);
      }
    }
    
    // === SLOT 6-11ms: IMU ===
    else if (cycle_time >= 6 && cycle_time < 11) {
#ifdef USE_BNO085
      if (imu.wasReset()) {
        LOG_W(LOG_IMU, "BNO085 was reset! Reconfiguring...");
        
        // Réactiver les reports après reset
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(500)) == pdTRUE) {
          bno085_enable_report(SH2_ROTATION_VECTOR, 20000);
          bno085_enable_report(SH2_LINEAR_ACCELERATION, 20000);
          bno085_enable_report(SH2_GYROSCOPE_CALIBRATED, 20000);
          xSemaphoreGive(i2c_mutex);
          LOG_I(LOG_IMU, "BNO085 reports reactivated");
        }
      }
      
      if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (imu.getSensorEvent(&sensor_value)) {
          // Sauvegarder selon le type de sensor
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
          
          // Calculer accélération verticale (BNO085: quaternion = r, i, j, k)
          // LINEAR_ACCELERATION est déjà sans gravité, ne pas soustraire
          acc_vertical = kalman_get_vertical_accel(quat_r, quat_i, quat_j, quat_k,
                                                    acc_x, acc_y, acc_z, false);
        }
        xSemaphoreGive(i2c_mutex);
      }
#else
      // Lecture LSM6DSO32 + fusion AHRS
      if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        sensors_event_t accel, gyro, temp;
        lsm6.getEvent(&accel, &gyro, &temp);
        
        // Sauvegarder valeurs brutes
        acc_x = accel.acceleration.x;
        acc_y = accel.acceleration.y;
        acc_z = accel.acceleration.z;
        gyro_x = gyro.gyro.x;
        gyro_y = gyro.gyro.y;
        gyro_z = gyro.gyro.z;
        
        xSemaphoreGive(i2c_mutex);
      }
      
      // Fusion AHRS (gyro en rad/s, acc en m/s²)
      // Sans magnétomètre: mettre 0,0,0 pour mx, my, mz
      filter.update(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
                    accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                    0, 0, 0);  // Pas de magnétomètre
      
      // Récupérer quaternion et angles d'Euler
      filter.getQuaternion(&quat_w, &quat_x, &quat_y, &quat_z);
      roll = filter.getRoll();
      pitch = filter.getPitch();
      yaw = filter.getYaw();
      
      // Calculer accélération verticale (LSM6: quaternion = w, x, y, z)
      // Accélération totale, il faut soustraire la gravité
      acc_vertical = kalman_get_vertical_accel(quat_w, quat_x, quat_y, quat_z,
                                                acc_x, acc_y, acc_z, true);
#endif
    }
    
    // === SLOT 12-16ms: GPS (tous les 5 cycles = 10Hz) ===
    else if (cycle_time >= 12 && cycle_time < 16 && counter % 5 == 0) {
      // Lire jusqu'à 200 caractères d'un coup (une trame NMEA complète)
      for (int i = 0; i < 200; i++) {
        if (!gps.read()) break;  // Sortir si plus de données
      }
      
      if (gps.newNMEAreceived()) {
        const char* nmea = gps.lastNMEA();
        
        // Sauvegarder pour log ultérieur
        strncpy(last_nmea, nmea, sizeof(last_nmea) - 1);
        last_nmea[sizeof(last_nmea) - 1] = '\0';
        
        // Parser (silencieux si trame non supportée comme GSV, GSA, VTG)
        gps.parse((char*)nmea);
        
        // Récupérer altitude GPS si fix valide
        if (gps.fix) {
          alt_gps = gps.altitude;
        } else {
          alt_gps = NAN;
        }
      }
    }
    
    // === SLOT 17-19ms: Battery (1x par seconde) ===
    else if (cycle_time >= 17 && counter % 50 == 0) {
      if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        last_voltage = battery.cellVoltage();
        last_percent = battery.cellPercent();
        xSemaphoreGive(i2c_mutex);
      }
    }
    
    // Vérifier stabilisation capteurs
    if (counter == STABILIZATION_CYCLES) {
      sensors_stabilized = true;
      LOG_I(LOG_SYSTEM, "Sensors stabilized (%d cycles), initializing Kalman...", STABILIZATION_CYCLES);
    }
    
    // Initialiser Kalman après stabilisation
    if (!kalman_initialized && sensors_stabilized) {
      kalman_init(&kalman, alt_baro);
      kalman_initialized = true;
    }
    
    // Fusion Kalman (si initialisé)
    if (kalman_initialized) {
      kalman_process(&kalman, acc_vertical, alt_baro, alt_gps);
      
      // === MISE À JOUR DES DONNÉES DE VOL ===
      if (flight_data_lock(10)) {
        // Altitudes
        flight_data.altitude_qne = kalman.altitude;
        flight_data.altitude_qnh = calc_altitude_qnh(bmp.pressure, flight_data.qnh_local);
        flight_data.altitude_gps = alt_gps;
        
        if (flight_data.flight_started) {
          flight_data.altitude_qfe = calc_altitude_qfe(kalman.altitude, 
                                                        flight_data.altitude_takeoff);
        }
        
        // Vario
        flight_data.vario = kalman.vario;
        flight_data.vario_integrated = calc_vario_integrated(kalman.vario);
        
        // GPS
        if (gps.fix) {
          flight_data.gps_fix = true;
          flight_data.latitude = gps.latitudeDegrees;
          flight_data.longitude = gps.longitudeDegrees;
          flight_data.speed_gps = gps.speed * 1.852f;  // Noeuds -> km/h
          flight_data.heading = gps.angle;
          flight_data.satellites = gps.satellites;
          
          // Calculs si vol démarré
          if (flight_data.flight_started) {
            // Distance depuis décollage
            flight_data.distance_takeoff = calc_distance_gps(
              flight_data.latitude_takeoff, flight_data.longitude_takeoff,
              flight_data.latitude, flight_data.longitude) / 1000.0f;  // m -> km
            
            // Cap vers décollage
            flight_data.heading_takeoff = calc_bearing_gps(
              flight_data.latitude, flight_data.longitude,
              flight_data.latitude_takeoff, flight_data.longitude_takeoff);
            
            // Finesse
            if (fabsf(kalman.vario) > 0.5f) {  // Seulement si on descend
              float speed_ms = gps.speed * 0.51444f;  // Noeuds -> m/s
              flight_data.glide_ratio = calc_glide_ratio(speed_ms, kalman.vario);
              flight_data.glide_ratio_avg = calc_glide_ratio_avg(flight_data.glide_ratio);
            }
          }
        } else {
          flight_data.gps_fix = false;
        }
        
        // Pression & Température
        flight_data.pressure = bmp.pressure;
        flight_data.temperature = bmp.temperature;
        
        // Batterie
        flight_data.battery_voltage = last_voltage;
        flight_data.battery_percent = last_percent;
        
        // G-Force
        flight_data.g_force = calc_g_force(acc_x, acc_y, acc_z);
        
        // Détection thermique
        flight_data.in_thermal = detect_thermal(kalman.vario, flight_data.in_thermal);
        if (flight_data.in_thermal) {
          // TODO: Calculer gain thermique
        }
        
        // Détection début de vol
        if (!flight_data.flight_started) {
          flight_data.flight_started = detect_flight_start(
            kalman.vario, flight_data.speed_gps, 
            kalman.altitude, flight_data.flight_started);
          
          if (flight_data.flight_started) {
            // Enregistrer position de décollage
            flight_data.altitude_takeoff = kalman.altitude;
            flight_data.latitude_takeoff = flight_data.latitude;
            flight_data.longitude_takeoff = flight_data.longitude;
            flight_data.flight_start_time = millis();
            LOG_I(LOG_FLIGHT, "Flight started!");
          }
        } else {
          // Temps de vol
          flight_data.flight_time = (millis() - flight_data.flight_start_time) / 1000;
        }
        
        // Mise à jour timestamp
        flight_data.last_update_ms = millis();
        
        flight_data_unlock();
      }
    }
    
    // Log à 1Hz
    if (counter % 50 == 0) {
      // GPS
      if (last_nmea[0] != '\0') {
        LOG_V(LOG_GPS, "%s", last_nmea);
      }
      
      // BMP585 (réutiliser les valeurs déjà lues pour éviter transaction I2C supplémentaire)
      if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (bmp.performReading()) {
          float temp = bmp.temperature;
          float pressure = bmp.pressure;  // Déjà en hPa
          
          LOG_V(LOG_BARO, "T=%.1f°C P=%.1fhPa Alt=%.1fm", temp, pressure, alt_baro);
        }
        xSemaphoreGive(i2c_mutex);
      }
      
      // Batterie (utiliser valeurs sauvegardées)
      LOG_V(LOG_BATTERY, "Battery: %.2fV (%.1f%%)", last_voltage, last_percent);
      
      // IMU
#ifdef USE_BNO085
      LOG_V(LOG_IMU, "Quat: i=%.3f j=%.3f k=%.3f r=%.3f Acc: x=%.2f y=%.2f z=%.2f Gyro: x=%.2f y=%.2f z=%.2f",
            quat_i, quat_j, quat_k, quat_r,
            acc_x, acc_y, acc_z,
            gyro_x, gyro_y, gyro_z);
#else
      LOG_V(LOG_IMU, "Quat: w=%.3f x=%.3f y=%.3f z=%.3f Euler: R=%.1f° P=%.1f° Y=%.1f° Acc: x=%.2f y=%.2f z=%.2f",
            quat_w, quat_x, quat_y, quat_z,
            roll, pitch, yaw,
            acc_x, acc_y, acc_z);
#endif

      // Kalman (résultat de fusion)
      if (kalman_initialized) {
        LOG_I(LOG_KALMAN, ">>> Alt=%.1fm Vario=%.2fm/s", kalman.altitude, kalman.vario);
      }
    }
    
    counter++;
    vTaskDelayUntil(&last_wake, freq);
  }
}

// =============================================================================
// FONCTIONS PUBLIQUES
// =============================================================================

/**
 * @brief Initialise I2C capteurs
 */
bool sensors_init_i2c() {
  LOG_I(LOG_SYSTEM, "Initializing I2C sensors...");
  
  // Créer le mutex pour protéger le bus I2C
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

/**
 * @brief Initialise BMP585
 */
bool sensors_init_bmp585() {
  LOG_I(LOG_BARO, "Initializing BMP585...");
  
  if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_BARO, "Failed to take I2C mutex");
    return false;
  }
  
  if (!bmp.begin(BMP5_I2C_ADDR_PRIM, &I2C_Sensors)) {
    LOG_E(LOG_BARO, "BMP585 not found at 0x%02X", BMP5_I2C_ADDR_PRIM);
    
    // Essayer adresse secondaire
    if (!bmp.begin(BMP5_I2C_ADDR_SEC, &I2C_Sensors)) {
      LOG_E(LOG_BARO, "BMP585 not found at 0x%02X", BMP5_I2C_ADDR_SEC);
      xSemaphoreGive(i2c_mutex);
      return false;
    }
  }
  
  // Configuration avec casts vers les bons types
  bmp.setTemperatureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_8X);
  bmp.setPressureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff((bmp5xx_iir_filter_t)BMP5_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate((bmp5xx_odr_t)BMP5_ODR_50_HZ);  // 50Hz
  
  xSemaphoreGive(i2c_mutex);
  
  LOG_I(LOG_BARO, "BMP585 OK (50Hz)");
  return true;
}

/**
 * @brief Initialise GPS PA1010D
 */
bool sensors_init_gps() {
  LOG_I(LOG_GPS, "Initializing GPS PA1010D...");
  
  if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_GPS, "Failed to take I2C mutex");
    return false;
  }
  
  if (!gps.begin(0x10)) {  // Adresse I2C standard PA1010D
    LOG_E(LOG_GPS, "GPS not found at 0x10");
    
    // Essayer adresse alternative
    if (!gps.begin(0x42)) {
      LOG_E(LOG_GPS, "GPS not found at 0x42");
      xSemaphoreGive(i2c_mutex);
      return false;
    }
  }
  
  xSemaphoreGive(i2c_mutex);
  
  delay(100);  // Attendre que le GPS soit prêt
  
  // Configuration PA1010D (MediaTek MT3333)
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // Seulement RMC + GGA
  delay(100);
  
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);     // 5Hz (max pour PA1010D)
  delay(100);
  
  gps.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);     // Fix à 5Hz
  delay(100);
  
  LOG_I(LOG_GPS, "GPS PA1010D OK (5Hz, RMC+GGA)");
  return true;
}

/**
 * @brief Initialise MAX17048 (fuel gauge batterie)
 */
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

/**
 * @brief Active un report sur le BNO085
 */
#ifdef USE_BNO085
static bool bno085_enable_report(sh2_SensorId_t sensor_id, uint32_t interval_us) {
  if (!imu.enableReport(sensor_id, interval_us)) {
    LOG_E(LOG_IMU, "Failed to enable report %d", sensor_id);
    return false;
  }
  return true;
}
#endif

/**
 * @brief Initialise IMU
 */
bool sensors_init_imu() {
#ifdef USE_BNO085
  LOG_I(LOG_IMU, "Initializing BNO085...");
  
  if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_IMU, "Failed to take I2C mutex");
    return false;
  }
  
  if (!imu.begin_I2C(BNO08x_I2CADDR_DEFAULT, &I2C_Sensors)) {
    LOG_E(LOG_IMU, "BNO085 not found at 0x%02X", BNO08x_I2CADDR_DEFAULT);
    
    // Essayer adresse alternative 0x4B
    if (!imu.begin_I2C(0x4B, &I2C_Sensors)) {
      LOG_E(LOG_IMU, "BNO085 not found at 0x4B");
      xSemaphoreGive(i2c_mutex);
      return false;
    }
  }
  
  LOG_D(LOG_IMU, "BNO085 found, configuring reports...");
  
  // Activer les reports nécessaires à 50Hz (20000 us)
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
    LOG_E(LOG_IMU, "LSM6DSO32 not found at 0x%02X", LSM6DS_I2CADDR_DEFAULT);
    
    // Essayer adresse alternative 0x6B
    if (!lsm6.begin_I2C(0x6B, &I2C_Sensors)) {
      LOG_E(LOG_IMU, "LSM6DSO32 not found at 0x6B");
      xSemaphoreGive(i2c_mutex);
      return false;
    }
  }
  
  LOG_D(LOG_IMU, "LSM6DSO32 found, configuring...");
  
  // Configuration haute performance
  lsm6.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  lsm6.setAccelDataRate(LSM6DS_RATE_104_HZ);  // 104Hz
  lsm6.setGyroDataRate(LSM6DS_RATE_104_HZ);
  
  xSemaphoreGive(i2c_mutex);
  
  // Initialiser le filtre AHRS (Madgwick) à 50Hz
  filter.begin(50);
  
  LOG_I(LOG_IMU, "LSM6DSO32 OK (104Hz, 16G, 1000dps, AHRS 50Hz)");
  return true;
#endif
}

/**
 * @brief Crée tâche capteurs
 * @param core Core sur lequel attacher la tâche (0 ou 1)
 */
bool sensors_create_task(BaseType_t core) {
  BaseType_t result = xTaskCreatePinnedToCore(
    task_sensors,
    "Sensors",
    4096,              // Stack size
    NULL,
    2,                 // Priorité
    &task_sensors_handle,
    core
  );

  if (result != pdPASS) {
    LOG_E(LOG_SYSTEM, "Sensors task creation failed");
    return false;
  }

  LOG_I(LOG_SYSTEM, "Sensors task created on core %d", core);
  return true;
}

/**
 * @brief Initialisation complète capteurs
 * @param core Core pour la tâche
 */
bool init_sensors_global(BaseType_t core) {
  // Initialiser structure de données de vol
  if (!flight_data_init()) {
    LOG_E(LOG_SYSTEM, "Failed to init flight data");
    return false;
  }
  
  if (!sensors_init_i2c()) {
    return false;
  }

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