/**
 * @file sensor_init.cpp
 * @brief Implémentation - VERSION GPS PA1010D I2C + BMP5
 *
 * Initialisation complète GPS + BMP5 via i2c_wrapper
 * 
 * Auteur : Franck Moreau
 */

#include "src/system/sensor_init/sensor_init.h"
#include "src/system/logger/logger.h"
#include "src/system/BMP5XX_ESP32/BMP5XX_ESP32.h"
#include "config/config.h"

// ================================
// === INSTANCES GLOBALES
// ================================
gps_device_t gps_dev;
gps_data_t gps_data;
bool sensor_gps_ready = false;

bmp5_device_t bmp5_dev;
bool sensor_bmp5_ready = false;

lsm6dso32_device_t lsm6dso32_dev;
bool sensor_imu_ready = false;

max17048_device_t max17048_dev;
bool sensor_battery_ready = false;

// ================================
// === SCAN I2C VIA WRAPPER
// ================================
uint8_t sensor_scan_i2c() {
  LOG_I(LOG_MODULE_SYSTEM, "Scanning I2C bus 1 (sensors)...");

  uint8_t count = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    if (i2c_probe_device(I2C_BUS_1, addr)) {
      LOG_I(LOG_MODULE_SYSTEM, "Device found at 0x%02X", addr);

      // Identifier les devices connus
      if (addr == GPS_I2C_ADDR) {
        LOG_I(LOG_MODULE_SYSTEM, "  -> GPS PA1010D detected");
      } else if (addr == LSM6DSO32_I2C_ADDR) {
        LOG_I(LOG_MODULE_SYSTEM, "  -> LSM6DSO32 detected (not used yet)");
      } else if (addr == BMP5_I2C_ADDR) {
        LOG_I(LOG_MODULE_SYSTEM, "  -> BMP5 detected");
      }

      count++;
    }
  }

  if (count == 0) {
    LOG_E(LOG_MODULE_SYSTEM, "No I2C devices found - check wiring!");
  } else {
    LOG_I(LOG_MODULE_SYSTEM, "Total devices found: %d", count);
  }

  return count;
}

// ================================
// === INIT GPS PA1010D
// ================================
bool sensor_init_gps() {
  LOG_I(LOG_MODULE_GPS, "Initializing GPS PA1010D...");

  gps_i2c_config_t config = {
    .bus = I2C_BUS_1,
    .address = GPS_I2C_ADDR
  };

  if (!GPS_init(&gps_dev, &config)) {
    LOG_E(LOG_MODULE_GPS, "GPS init failed");
    sensor_gps_ready = false;
    return false;
  }

  LOG_I(LOG_MODULE_GPS, "GPS PA1010D initialized successfully");
  sensor_gps_ready = true;
  return true;
}

// ================================
// === INIT BMP5
// ================================
bool sensor_init_bmp5() {
  LOG_I(LOG_MODULE_BMP5, "Init BMP5 (simple driver)...");

  bmp5_config_t config = {
    .bus = I2C_BUS_1,
    .address = BMP5_I2C_ADDR,
    .osr_temp = BMP5_OSR_2X,
    .osr_press = BMP5_OSR_16X,
    .iir_filter = BMP5_IIR_COEFF_3,
    .odr = BMP5_ODR_50_HZ
  };

  if (!BMP5_init(&bmp5_dev, &config)) {
    return false;
  }

  sensor_bmp5_ready = true;
  return true;
}

bool sensor_init_imu() {
  LOG_I(LOG_MODULE_IMU, "Initializing LSM6DSO32...");

  lsm6dso32_config_t config = {
    .bus = I2C_BUS_1,
    .address = LSM6DSO32_I2C_ADDR,
    .accel_range = LSM6DSO32_ACCEL_RANGE_8G,     // ±8G pour parapente
    .gyro_range = LSM6DSO32_GYRO_RANGE_125_DPS,  // ±125dps max précision
    .accel_rate = LSM6DSO32_RATE_104_HZ,         // 104 Hz
    .gyro_rate = LSM6DSO32_RATE_104_HZ           // 104 Hz
  };

  if (!LSM6DSO32_init(&lsm6dso32_dev, &config)) {
    return false;
  }

  sensor_imu_ready = true;
  return true;
}

bool sensor_init_battery() {
  LOG_I(LOG_MODULE_SYSTEM, "Initializing MAX17048 battery gauge...");

  max17048_config_t config = {
    .bus = I2C_BUS_1,
    .address = MAX17048_I2C_ADDR,
    .alert_threshold = 10  // Alerte à 10%
  };

  if (!MAX17048_init(&max17048_dev, &config)) {
    return false;
  }

  sensor_battery_ready = true;
  return true;
}

/**
 * @brief Lit et parse un caractère GPS
 * 
 * À appeler en boucle (polling). Lit un caractère, construit
 * les lignes NMEA et les parse quand complètes.
 * 
 * @return true si ligne NMEA parsée avec succès, false sinon
 */
bool sensor_read_gps() {
  if (!sensor_gps_ready) return false;

  return GPS_read(&gps_dev, &gps_data);
}

/**
 * @brief Lit température, pression et calcule altitude
 * 
 * Effectue une lecture immédiate du BMP5.
 * Calcul altitude basé sur QNH 1013.25 hPa.
 * 
 * @return true si lecture réussie, false sinon
 */
bool sensor_read_bmp5() {
  if (!sensor_bmp5_ready) return false;

  bmp5_data_t data;
  if (BMP5_read(&bmp5_dev, &data, 1013.25f)) {
    //LOG_I(LOG_MODULE_BMP5, "T=%.2f°C P=%.2fhPa Alt=%.1fm",
    //      data.temperature, data.pressure, data.altitude);
    return true;
  }

  return false;
}

/**
 * @brief Lit BMP5 en mode optimisé (pression seule)
 * 
 * Pour variomètre : on lit souvent juste la pression.
 * Économise 50% des données I2C vs lecture complète.
 * 
 * @return true si succès
 */
bool sensor_read_bmp5_fast() {
  if (!sensor_bmp5_ready) return false;

  float pressure;
  if (BMP5_read_pressure_only(&bmp5_dev, &pressure)) {
    LOG_V(LOG_MODULE_BMP5, "P=%.2f hPa (fast)", pressure);
    return true;
  }

  return false;
}

bool sensor_read_imu() {
  if (!sensor_imu_ready) return false;

  lsm6dso32_data_t data;
  if (LSM6DSO32_read(&lsm6dso32_dev, &data)) {
    LOG_V(LOG_MODULE_IMU, "A[%.2f %.2f %.2f] G[%.3f %.3f %.3f] T=%.1f°C",
          data.accel_x, data.accel_y, data.accel_z,
          data.gyro_x, data.gyro_y, data.gyro_z,
          data.temperature);
    return true;
  }

  return false;
}

bool sensor_read_battery() {
  if (!sensor_battery_ready) return false;

  max17048_data_t data;
  if (MAX17048_read(&max17048_dev, &data)) {
    LOG_V(LOG_MODULE_SYSTEM, "Battery: %.2fV (%.1f%%) %s",
          data.voltage, data.soc,
          data.is_charging ? "CHARGING" : "");
    return true;
  }

  return false;
}

// ================================
// === TEST GPS (affiche statut)
// ================================
void sensor_test_gps() {
  if (!sensor_gps_ready) {
    LOG_E(LOG_MODULE_GPS, "GPS not ready for testing");
    return;
  }

  LOG_I(LOG_MODULE_GPS, "");
  LOG_I(LOG_MODULE_GPS, "=== GPS Status ===");
  LOG_I(LOG_MODULE_GPS, "Fix: %s", gps_data.fix ? "YES" : "NO");
  LOG_I(LOG_MODULE_GPS, "Satellites: %d", gps_data.satellites);

  if (gps_data.fix) {
    LOG_I(LOG_MODULE_GPS, "Position: %.6f, %.6f",
          gps_data.latitude, gps_data.longitude);
    LOG_I(LOG_MODULE_GPS, "Altitude: %.1f m", gps_data.altitude);
    LOG_I(LOG_MODULE_GPS, "Speed: %.1f knots", gps_data.speed);
    LOG_I(LOG_MODULE_GPS, "Course: %.1f°", gps_data.course);
    LOG_I(LOG_MODULE_GPS, "HDOP: %.1f", gps_data.hdop);

    // Temps depuis dernière mise à jour
    float elapsed = GPS_time_since_update(&gps_data);
    LOG_I(LOG_MODULE_GPS, "Last update: %.1f s ago", elapsed);
  } else {
    LOG_I(LOG_MODULE_GPS, "Waiting for GPS fix...");
  }
  // Temps
  LOG_I(LOG_MODULE_GPS, "Time: %02d:%02d:%02d.%03d UTC",
        gps_data.hour, gps_data.minute, gps_data.second, gps_data.millisecond);
  LOG_I(LOG_MODULE_GPS, "Date: %02d/%02d/20%02d",
        gps_data.day, gps_data.month, gps_data.year);

  LOG_I(LOG_MODULE_GPS, "==================");
  LOG_I(LOG_MODULE_GPS, "");
}
// ================================
// === TEST BMP5 (affiche statut)
// ================================
void sensor_test_bmp5() {
  if (!sensor_bmp5_ready) {
    LOG_E(LOG_MODULE_BMP5, "BMP5 not ready for testing");
    return;
  }

  LOG_I(LOG_MODULE_BMP5, "");
  LOG_I(LOG_MODULE_BMP5, "=== BMP5 Status ===");

  // Lire les données
  bmp5_data_t data;
  if (BMP5_read(&bmp5_dev, &data, 1013.25f)) {
    LOG_I(LOG_MODULE_BMP5, "Temperature: %.2f °C", data.temperature);
    LOG_I(LOG_MODULE_BMP5, "Pressure: %.2f hPa", data.pressure);
    LOG_I(LOG_MODULE_BMP5, "Altitude (QNH 1013.25): %.1f m", data.altitude);
  } else {
    LOG_E(LOG_MODULE_BMP5, "Read failed!");
  }

  LOG_I(LOG_MODULE_BMP5, "=====================");
  LOG_I(LOG_MODULE_BMP5, "");
}

void sensor_test_imu() {
  if (!sensor_imu_ready) {
    LOG_E(LOG_MODULE_IMU, "IMU not ready");
    return;
  }

  LOG_I(LOG_MODULE_IMU, "");
  LOG_I(LOG_MODULE_IMU, "=== LSM6DSO32 Status ===");

  lsm6dso32_data_t data;
  if (LSM6DSO32_read(&lsm6dso32_dev, &data)) {
    LOG_I(LOG_MODULE_IMU, "Accel: X=%.2f Y=%.2f Z=%.2f m/s²",
          data.accel_x, data.accel_y, data.accel_z);
    LOG_I(LOG_MODULE_IMU, "Gyro:  X=%.3f Y=%.3f Z=%.3f rad/s",
          data.gyro_x, data.gyro_y, data.gyro_z);
    LOG_I(LOG_MODULE_IMU, "Temp: %.1f°C", data.temperature);
  } else {
    LOG_E(LOG_MODULE_IMU, "Read failed!");
  }

  LOG_I(LOG_MODULE_IMU, "========================");
  LOG_I(LOG_MODULE_IMU, "");
}

void sensor_test_battery() {
  if (!sensor_battery_ready) {
    LOG_E(LOG_MODULE_SYSTEM, "Battery gauge not ready");
    return;
  }

  LOG_I(LOG_MODULE_SYSTEM, "");
  LOG_I(LOG_MODULE_SYSTEM, "=== MAX17048 Status ===");

  max17048_data_t data;
  if (MAX17048_read(&max17048_dev, &data)) {
    LOG_I(LOG_MODULE_SYSTEM, "Voltage: %.2f V", data.voltage);
    LOG_I(LOG_MODULE_SYSTEM, "SOC: %.1f %%", data.soc);
    LOG_I(LOG_MODULE_SYSTEM, "Charge rate: %.2f %%/h", data.charge_rate);
    LOG_I(LOG_MODULE_SYSTEM, "Status: %s",
          data.is_charging ? "CHARGING" : "DISCHARGING");

    if (data.alert) {
      LOG_W(LOG_MODULE_SYSTEM, "LOW BATTERY ALERT!");
    }
  } else {
    LOG_E(LOG_MODULE_SYSTEM, "Read failed!");
  }

  LOG_I(LOG_MODULE_SYSTEM, "=======================");
  LOG_I(LOG_MODULE_SYSTEM, "");
}

// ================================
// === INIT GLOBAL (GPS + BMP5)
// ================================
bool sensor_init_all() {
  LOG_I(LOG_MODULE_SYSTEM, "");
  LOG_I(LOG_MODULE_SYSTEM, "=== Sensor Initialization ===");

  // Scan I2C (debug)
  uint8_t devices_found = sensor_scan_i2c();
  if (devices_found == 0) {
    LOG_E(LOG_MODULE_SYSTEM, "No I2C devices found on sensor bus");
    return false;
  }

  LOG_I(LOG_MODULE_SYSTEM, "");

  // ✅ Initialiser GPS
  bool gps_ok = sensor_init_gps();

  LOG_I(LOG_MODULE_SYSTEM, "");

  // ✅ Initialiser BMP5
  bool bmp5_ok = sensor_init_bmp5();

  LOG_I(LOG_MODULE_SYSTEM, "");

  // ✅ Initialiser IMU
  bool imu_ok = sensor_init_imu();

  LOG_I(LOG_MODULE_SYSTEM, "");

  // ✅ Initialiser batterie
  bool battery_ok = sensor_init_battery();

  LOG_I(LOG_MODULE_SYSTEM, "");

  // Afficher résumé
  sensor_init_print_summary();

  LOG_I(LOG_MODULE_SYSTEM, "=== Init Complete ===");
  LOG_I(LOG_MODULE_SYSTEM, "");

  return gps_ok || bmp5_ok || imu_ok || battery_ok;  // Au moins un capteur OK
}

// ================================
// === SUMMARY
// ================================
void sensor_init_print_summary() {
  LOG_I(LOG_MODULE_SYSTEM, "");
  LOG_I(LOG_MODULE_SYSTEM, "--- Sensors Summary ---");
  LOG_I(LOG_MODULE_SYSTEM, "GPS PA1010D : %s",
        sensor_gps_ready ? "OK" : "FAIL");
  LOG_I(LOG_MODULE_SYSTEM, "BMP5        : %s",
        sensor_bmp5_ready ? "OK" : "FAIL");
  LOG_I(LOG_MODULE_SYSTEM, "LSM6DSO32   : %s",  // ✅ AJOUT
        sensor_imu_ready ? "OK" : "FAIL");
  LOG_I(LOG_MODULE_SYSTEM, "MAX17048    : %s",
        sensor_battery_ready ? "OK" : "FAIL");
  LOG_I(LOG_MODULE_SYSTEM, "----------------------");
  LOG_I(LOG_MODULE_SYSTEM, "");
}
