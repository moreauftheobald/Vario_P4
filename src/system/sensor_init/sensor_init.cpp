/**
 * @file sensor_init.cpp
 * @brief Implémentation - VERSION GPS PA1010D I2C + BMP5 + IMU (BNO08x ou LSM6DSO32)
 *
 * Initialisation complète GPS + BMP5 + IMU via i2c_wrapper
 * 
 * Auteur : Franck Moreau
 */

#include "src/system/sensor_init/sensor_init.h"
#include "src/system/logger/logger.h"
#include "src/system/BMP5XX_ESP32/BMP5XX_ESP32.h"
#include "config/config.h"
#include "config/pins.h"

// ================================
// === INSTANCES GLOBALES
// ================================
gps_device_t gps_dev;
gps_data_t gps_data;
bool sensor_gps_ready = false;

bmp5_device_t bmp5_dev;
bool sensor_bmp5_ready = false;

#if IMU_BNO08XX == 1
BNO08x_ESP32_P4* bno08x_dev = nullptr;
#else
lsm6dso32_device_t lsm6dso32_dev;
#endif
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
      } else if (addr == BNO08X_I2C_ADDR) {
        LOG_I(LOG_MODULE_SYSTEM, "  -> BNO08x detected");
      } else if (addr == LSM6DSO32_I2C_ADDR) {
        LOG_I(LOG_MODULE_SYSTEM, "  -> LSM6DSO32 detected");
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

// ================================
// === INIT IMU (BNO08x ou LSM6DSO32)
// ================================
bool sensor_init_imu() {
/*#if IMU_BNO08XX == 1
  LOG_I(LOG_MODULE_IMU, "Initializing BNO08x...");

  // Créer l'instance C++ (pas de pin reset dans constructeur)
  bno08x_dev = new BNO08x_ESP32_P4(-1);  // -1 = pas de reset HW
  if (!bno08x_dev) {
    LOG_E(LOG_MODULE_IMU, "Failed to allocate BNO08x");
    return false;
  }

  // ✅ LOG : Vérifier quel bus on utilise
  LOG_I(LOG_MODULE_IMU, "Using I2C_BUS_%d for BNO08x", I2C_BUS_1);

  // ✅ Initialiser sur I2C_BUS_1 (uint8_t 1)
  if (!bno08x_dev->begin_I2C(I2C_BUS_1, BNO08X_I2C_ADDR, 0)) {
    LOG_E(LOG_MODULE_IMU, "BNO08x init failed");
    delete bno08x_dev;
    bno08x_dev = nullptr;
    return false;
  }

  // Activer les capteurs nécessaires au vario
  if (!bno08x_dev->enableReport(SH2_ACCELEROMETER, 5000)) {  // 200Hz
    LOG_W(LOG_MODULE_IMU, "Failed to enable accelerometer");
  }

  if (!bno08x_dev->enableReport(SH2_GYROSCOPE_CALIBRATED, 5000)) {  // 200Hz
    LOG_W(LOG_MODULE_IMU, "Failed to enable gyroscope");
  }

  if (!bno08x_dev->enableReport(SH2_ROTATION_VECTOR, 10000)) {  // 100Hz
    LOG_W(LOG_MODULE_IMU, "Failed to enable rotation vector");
  }

  if (!bno08x_dev->enableReport(SH2_LINEAR_ACCELERATION, 10000)) {  // 100Hz
    LOG_W(LOG_MODULE_IMU, "Failed to enable linear acceleration");
  }

  // ✅ Test polling immédiat
  LOG_I(LOG_MODULE_IMU, "Testing sensor data polling (waiting 500ms)...");
  delay(500);

  extern void sh2_service(void);
  sh2_SensorValue_t test_event;
  int events_found = 0;

  for (int attempt = 0; attempt < 20; attempt++) {
    sh2_service();
    delay(10);

    if (bno08x_dev->getSensorEvent(&test_event)) {
      events_found++;
      LOG_I(LOG_MODULE_IMU, "Event %d: sensorId=0x%02X timestamp=%lu",
            events_found, test_event.sensorId, test_event.timestamp);

      if (events_found >= 5) break;
    }
  }

  if (events_found == 0) {
    LOG_W(LOG_MODULE_IMU, "WARNING: No sensor events received!");
  }

  LOG_I(LOG_MODULE_IMU, "BNO08x initialized successfully");
  sensor_imu_ready = true;
  return true;

#else
  // LSM6DSO32
  LOG_I(LOG_MODULE_IMU, "Initializing LSM6DSO32...");

  lsm6dso32_config_t config = {
    .bus = I2C_BUS_1,
    .address = LSM6DSO32_I2C_ADDR,
    .accel_range = LSM6DSO32_ACCEL_RANGE_8G,
    .gyro_range = LSM6DSO32_GYRO_RANGE_125_DPS,
    .accel_rate = LSM6DSO32_RATE_104_HZ,
    .gyro_rate = LSM6DSO32_RATE_104_HZ
  };

  if (!LSM6DSO32_init(&lsm6dso32_dev, &config)) {
    return false;
  }

  sensor_imu_ready = true;
  return true;
#endif*/
}

// ================================
// === INIT BATTERY
// ================================
bool sensor_init_battery() {
  LOG_I(LOG_MODULE_SYSTEM, "Initializing MAX17048 battery gauge...");

  max17048_config_t config = {
    .bus = I2C_BUS_1,
    .address = MAX17048_I2C_ADDR,
    .alert_threshold = 10
  };

  if (!MAX17048_init(&max17048_dev, &config)) {
    return false;
  }

  sensor_battery_ready = true;
  return true;
}

// ================================
// === READ SENSORS
// ================================

bool sensor_read_gps() {
  if (!sensor_gps_ready) return false;
  return GPS_read(&gps_dev, &gps_data);
}

bool sensor_read_bmp5() {
  if (!sensor_bmp5_ready) return false;

  bmp5_data_t data;
  if (BMP5_read(&bmp5_dev, &data, 1013.25f)) {
    return true;
  }

  return false;
}

bool sensor_read_bmp5_fast() {
  if (!sensor_bmp5_ready) return false;

  float pressure;
  return BMP5_read_pressure_only(&bmp5_dev, &pressure);
}

bool sensor_read_imu() {
  if (!sensor_imu_ready) return false;

#if IMU_BNO08XX == 1
  // Traité dans task_flight.h via getSensorEvent()
  return true;
#else
  lsm6dso32_data_t data;
  return LSM6DSO32_read(&lsm6dso32_dev, &data);
#endif
}

bool sensor_read_battery() {
  if (!sensor_battery_ready) return false;

  max17048_data_t data;
  return MAX17048_read(&max17048_dev, &data);
}

// ================================
// === TEST FUNCTIONS
// ================================

void sensor_test_gps() {
  if (!sensor_gps_ready) {
    LOG_W(LOG_MODULE_GPS, "GPS not ready");
    return;
  }

  LOG_I(LOG_MODULE_GPS, "Fix: %d | Sats: %d", gps_data.fix, gps_data.satellites);
  if (gps_data.fix) {
    LOG_I(LOG_MODULE_GPS, "Lat: %.6f | Lon: %.6f | Alt: %.1fm",
          gps_data.latitude, gps_data.longitude, gps_data.altitude);
  }
}

void sensor_test_bmp5() {
  if (!sensor_bmp5_ready) {
    LOG_W(LOG_MODULE_BMP5, "BMP5 not ready");
    return;
  }

  bmp5_data_t data;
  if (BMP5_read(&bmp5_dev, &data, 1013.25f)) {
    LOG_I(LOG_MODULE_BMP5, "T=%.2f°C P=%.2fhPa Alt=%.1fm",
          data.temperature, data.pressure, data.altitude);
  }
}

void sensor_test_imu() {
  if (!sensor_imu_ready) {
    LOG_W(LOG_MODULE_IMU, "IMU not ready");
    return;
  }

/*#if IMU_BNO08XX == 1
  sh2_SensorValue_t event;
  if (bno08x_dev->getSensorEvent(&event)) {
    if (event.sensorId == SH2_ACCELEROMETER) {
      LOG_I(LOG_MODULE_IMU, "Accel: X=%.3f Y=%.3f Z=%.3f m/s²",
            event.un.accelerometer.x,
            event.un.accelerometer.y,
            event.un.accelerometer.z);
    }
  }
#else
  lsm6dso32_data_t data;
  if (LSM6DSO32_read(&lsm6dso32_dev, &data)) {
    LOG_I(LOG_MODULE_IMU, "Accel: X=%.3f Y=%.3f Z=%.3f m/s²",
          data.accel_x, data.accel_y, data.accel_z);
    LOG_I(LOG_MODULE_IMU, "Gyro: X=%.3f Y=%.3f Z=%.3f rad/s",
          data.gyro_x, data.gyro_y, data.gyro_z);
  }
#endif*/
}

void sensor_test_battery() {
  if (!sensor_battery_ready) {
    LOG_W(LOG_MODULE_SYSTEM, "Battery gauge not ready");
    return;
  }

  max17048_data_t data;
  if (MAX17048_read(&max17048_dev, &data)) {
    LOG_I(LOG_MODULE_SYSTEM, "Battery: %.1f%% | %.2fV",
          data.soc, data.voltage);
  }
}

// ================================
// === INIT ALL & SUMMARY
// ================================

bool sensor_init_all() {
  bool gps_ok = sensor_init_gps();
  bool bmp_ok = sensor_init_bmp5();
  bool imu_ok = sensor_init_imu();
  bool bat_ok = sensor_init_battery();

  return (gps_ok || bmp_ok || imu_ok || bat_ok);
}

void sensor_init_print_summary() {
  LOG_I(LOG_MODULE_SYSTEM, "");
  LOG_I(LOG_MODULE_SYSTEM, "=== Sensors Status ===");
  LOG_I(LOG_MODULE_SYSTEM, "GPS PA1010D : %s", sensor_gps_ready ? "OK" : "FAIL");
  LOG_I(LOG_MODULE_SYSTEM, "BMP5        : %s", sensor_bmp5_ready ? "OK" : "FAIL");
#if IMU_BNO08XX == 1
  LOG_I(LOG_MODULE_SYSTEM, "BNO08x      : %s", sensor_imu_ready ? "OK" : "FAIL");
#else
  LOG_I(LOG_MODULE_SYSTEM, "LSM6DSO32   : %s", sensor_imu_ready ? "OK" : "FAIL");
#endif
  LOG_I(LOG_MODULE_SYSTEM, "MAX17048    : %s", sensor_battery_ready ? "OK" : "FAIL");
  LOG_I(LOG_MODULE_SYSTEM, "=====================");
}