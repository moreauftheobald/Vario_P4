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
  const TickType_t freq = pdMS_TO_TICKS(20);  // 50Hz
  uint32_t counter = 0;
  static char last_nmea[120] = "";

  while (true) {
    // Lecture GPS (parser les caractères disponibles)
    char c = gps.read();
    if (gps.newNMEAreceived()) {
      const char* nmea = gps.lastNMEA();
      
      // Sauvegarder pour log ultérieur
      strncpy(last_nmea, nmea, sizeof(last_nmea) - 1);
      last_nmea[sizeof(last_nmea) - 1] = '\0';
      
      // Parser (silencieux si trame non supportée comme GSV, GSA, VTG)
      gps.parse((char*)nmea);
    }
    
    // Lecture IMU
#ifdef USE_BNO085
    if (imu.wasReset()) {
      LOG_W(LOG_IMU, "BNO085 was reset!");
    }
    
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
    }
#else
    // Lecture LSM6DSO32 + fusion AHRS
    sensors_event_t accel, gyro, temp;
    lsm6.getEvent(&accel, &gyro, &temp);
    
    // Sauvegarder valeurs brutes
    acc_x = accel.acceleration.x;
    acc_y = accel.acceleration.y;
    acc_z = accel.acceleration.z;
    gyro_x = gyro.gyro.x;
    gyro_y = gyro.gyro.y;
    gyro_z = gyro.gyro.z;
    
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
#endif
    
    // Log à 1Hz
    if (counter % 50 == 0) {
      // GPS
      if (last_nmea[0] != '\0') {
        LOG_V(LOG_GPS, "%s", last_nmea);
      }
      
      // BMP585
      if (bmp.performReading()) {
        float temp = bmp.temperature;
        float pressure = bmp.pressure;  // Déjà en hPa
        float altitude = bmp.readAltitude(1013.25);  // QNE
        
        LOG_V(LOG_BARO, "T=%.1f°C P=%.1fhPa Alt=%.1fm", temp, pressure, altitude);
      }
      
      // Batterie
      float voltage = battery.cellVoltage();
      float percent = battery.cellPercent();
      LOG_V(LOG_BATTERY, "Battery: %.2fV (%.1f%%)", voltage, percent);
      
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
    }
    
    // TODO: Fusion Kalman
    
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
  
  if (!bmp.begin(BMP5_I2C_ADDR_PRIM, &I2C_Sensors)) {
    LOG_E(LOG_BARO, "BMP585 not found at 0x%02X", BMP5_I2C_ADDR_PRIM);
    
    // Essayer adresse secondaire
    if (!bmp.begin(BMP5_I2C_ADDR_SEC, &I2C_Sensors)) {
      LOG_E(LOG_BARO, "BMP585 not found at 0x%02X", BMP5_I2C_ADDR_SEC);
      return false;
    }
  }
  
  // Configuration avec casts vers les bons types
  bmp.setTemperatureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_8X);
  bmp.setPressureOversampling((bmp5xx_oversampling_t)BMP5_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff((bmp5xx_iir_filter_t)BMP5_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate((bmp5xx_odr_t)BMP5_ODR_50_HZ);
  
  LOG_I(LOG_BARO, "BMP585 OK");
  return true;
}

/**
 * @brief Initialise GPS PA1010D
 */
bool sensors_init_gps() {
  LOG_I(LOG_GPS, "Initializing GPS PA1010D...");
  
  if (!gps.begin(0x10)) {  // Adresse I2C standard PA1010D
    LOG_E(LOG_GPS, "GPS not found at 0x10");
    
    // Essayer adresse alternative
    if (!gps.begin(0x42)) {
      LOG_E(LOG_GPS, "GPS not found at 0x42");
      return false;
    }
  }
  
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
  
  if (!battery.begin(&I2C_Sensors)) {
    LOG_E(LOG_BATTERY, "MAX17048 not found");
    return false;
  }
  
  LOG_I(LOG_BATTERY, "MAX17048 OK (%.2fV, %.1f%%)", 
        battery.cellVoltage(), battery.cellPercent());
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
  
  if (!imu.begin_I2C(BNO08x_I2CADDR_DEFAULT, &I2C_Sensors)) {
    LOG_E(LOG_IMU, "BNO085 not found at 0x%02X", BNO08x_I2CADDR_DEFAULT);
    
    // Essayer adresse alternative 0x4B
    if (!imu.begin_I2C(0x4B, &I2C_Sensors)) {
      LOG_E(LOG_IMU, "BNO085 not found at 0x4B");
      return false;
    }
  }
  
  LOG_D(LOG_IMU, "BNO085 found, configuring reports...");
  
  // Activer les reports nécessaires à 50Hz (20000 us)
  if (!bno085_enable_report(SH2_ROTATION_VECTOR, 20000)) return false;           // Quaternion
  if (!bno085_enable_report(SH2_LINEAR_ACCELERATION, 20000)) return false;       // Accélération linéaire
  if (!bno085_enable_report(SH2_GYROSCOPE_CALIBRATED, 20000)) return false;      // Gyroscope
  
  LOG_I(LOG_IMU, "BNO085 OK (50Hz, Quat+Acc+Gyro)");
  return true;
  
#else
  LOG_I(LOG_IMU, "Initializing LSM6DSO32...");
  
  if (!lsm6.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &I2C_Sensors)) {
    LOG_E(LOG_IMU, "LSM6DSO32 not found at 0x%02X", LSM6DS_I2CADDR_DEFAULT);
    
    // Essayer adresse alternative 0x6B
    if (!lsm6.begin_I2C(0x6B, &I2C_Sensors)) {
      LOG_E(LOG_IMU, "LSM6DSO32 not found at 0x6B");
      return false;
    }
  }
  
  LOG_D(LOG_IMU, "LSM6DSO32 found, configuring...");
  
  // Configuration haute performance
  lsm6.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  lsm6.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6.setGyroDataRate(LSM6DS_RATE_104_HZ);
  
  // Initialiser le filtre AHRS (Madgwick)
  filter.begin(50);  // 50Hz
  
  LOG_I(LOG_IMU, "LSM6DSO32 OK (104Hz, 16G, 1000dps, AHRS Madgwick)");
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