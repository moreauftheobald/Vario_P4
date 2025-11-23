/**
 * @file sensor_init.cpp
 * @brief Initialisation centralisée capteurs - GPS PA1010D I2C + BMP5 + IMU + Battery
 * 
 * @author Franck Moreau
 * @date 2025-11-23
 * @version 1.0
 */

#include "sensor_init.h"
#include "src/system/logger/logger.h"
#include "config/config.h"

// =============================================================================
// INSTANCES GLOBALES
// =============================================================================

// GPS PA1010D I2C
gps_device_t gps_dev;
gps_data_t gps_data;
bool sensor_gps_ready = false;

// BMP5 Baromètre
bmp5_device_t bmp5_dev;
bool sensor_bmp5_ready = false;

// IMU (selon config)
#if IMU_BNO08XX == 1
  BNO08x_ESP32_P4* bno08x_dev = nullptr;
#else
  lsm6dso32_device_t lsm6dso32_dev;
#endif
bool sensor_imu_ready = false;

// Battery
max17048_device_t max17048_dev;
bool sensor_battery_ready = false;

// =============================================================================
// SCAN I2C
// =============================================================================

uint8_t sensor_scan_i2c() {
  LOG_I(LOG_MODULE_SYSTEM, "Scanning I2C bus 1...");
  
  uint8_t count = 0;
  i2c_bus_id_t bus = I2C_BUS_1;
  
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    if (i2c_lock(bus, 100)) {
      i2c_cmd_handle_t cmd = i2c_cmd_link_create();
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
      i2c_master_stop(cmd);
      
      i2c_port_t port = (bus == I2C_BUS_1) ? I2C_NUM_1 : I2C_NUM_0;
      esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
      i2c_cmd_link_delete(cmd);
      i2c_unlock(bus);
      
      if (ret == ESP_OK) {
        LOG_I(LOG_MODULE_SYSTEM, "Found device at 0x%02X", addr);
        count++;
      }
    }
  }
  
  LOG_I(LOG_MODULE_SYSTEM, "Scan complete: %d devices found", count);
  return count;
}

// =============================================================================
// INIT GPS
// =============================================================================

bool sensor_init_gps() {
  LOG_I(LOG_MODULE_GPS, "Init GPS PA1010D");

  gps_i2c_config_t config = {
    .bus = I2C_BUS_1,
    .address = GPS_I2C_ADDR
  };

  if (!GPS_init(&gps_dev, &config)) {
    LOG_E(LOG_MODULE_GPS, "Init failed");
    return false;
  }

  LOG_I(LOG_MODULE_GPS, "GPS OK");
  sensor_gps_ready = true;
  return true;
}

// =============================================================================
// INIT BMP5
// =============================================================================

bool sensor_init_bmp5() {
  LOG_I(LOG_MODULE_BMP5, "Init BMP5");

  bmp5_config_t config = {
    .bus = I2C_BUS_1,
    .address = BMP5_I2C_ADDR,
    .osr_temp = BMP5_TEMP_OVERSAMPLE,
    .osr_press = BMP5_PRESS_OVERSAMPLE,
    .iir_filter = BMP5_IIR_FILTER,
    .odr = BMP5_OUTPUT_DATA_RATE
  };

  if (!BMP5_init(&bmp5_dev, &config)) {
    LOG_E(LOG_MODULE_BMP5, "Init failed");
    return false;
  }

  LOG_I(LOG_MODULE_BMP5, "BMP5 OK");
  sensor_bmp5_ready = true;
  return true;
}

// =============================================================================
// INIT IMU (BNO08x ou LSM6DSO32)
// =============================================================================

bool sensor_init_imu() {
#if IMU_BNO08XX == 1
  LOG_I(LOG_MODULE_IMU, "Init BNO08x");

  bno08x_dev = new BNO08x_ESP32_P4();
  if (!bno08x_dev) {
    LOG_E(LOG_MODULE_IMU, "Allocation failed");
    return false;
  }

  if (!bno08x_dev->begin(I2C_BUS_1, BNO08X_I2C_ADDR)) {
    LOG_E(LOG_MODULE_IMU, "Init failed");
    delete bno08x_dev;
    bno08x_dev = nullptr;
    return false;
  }

  // Activer uniquement rotation vector (quaternions, 100Hz)
  if (!bno08x_dev->enableRotationVector(10000)) {  // 10000µs = 100Hz
    LOG_W(LOG_MODULE_IMU, "Failed to enable rotation vector");
  }

  LOG_I(LOG_MODULE_IMU, "BNO08x OK");
  sensor_imu_ready = true;
  return true;

#else
  LOG_I(LOG_MODULE_IMU, "Init LSM6DSO32");

  lsm6dso32_config_t config = {
    .bus = I2C_BUS_1,
    .address = LSM6DSO32_I2C_ADDR,
    .accel_range = LSM6DSO32_ACCEL_RANGE_8G,
    .gyro_range = LSM6DSO32_GYRO_RANGE_125_DPS,
    .accel_rate = LSM6DSO32_RATE_104_HZ,
    .gyro_rate = LSM6DSO32_RATE_104_HZ
  };

  if (!LSM6DSO32_init(&lsm6dso32_dev, &config)) {
    LOG_E(LOG_MODULE_IMU, "Init failed");
    return false;
  }

  LOG_I(LOG_MODULE_IMU, "LSM6DSO32 OK");
  sensor_imu_ready = true;
  return true;
#endif
}

// =============================================================================
// INIT BATTERY
// =============================================================================

bool sensor_init_battery() {
  LOG_I(LOG_MODULE_SYSTEM, "Init MAX17048");

  max17048_config_t config = {
    .bus = I2C_BUS_1,
    .address = MAX17048_I2C_ADDR,
    .alert_threshold = 10
  };

  if (!MAX17048_init(&max17048_dev, &config)) {
    LOG_E(LOG_MODULE_SYSTEM, "Init failed");
    return false;
  }

  LOG_I(LOG_MODULE_SYSTEM, "MAX17048 OK");
  sensor_battery_ready = true;
  return true;
}

// =============================================================================
// READ SENSORS
// =============================================================================

bool sensor_read_gps() {
  if (!sensor_gps_ready) return false;
  return GPS_read(&gps_dev, &gps_data);
}

bool sensor_read_bmp5() {
  if (!sensor_bmp5_ready) return false;

  bmp5_data_t data;
  return BMP5_read(&bmp5_dev, &data, 1013.25f);
}

bool sensor_read_bmp5_fast() {
  if (!sensor_bmp5_ready) return false;

  float pressure;
  return BMP5_read_pressure_only(&bmp5_dev, &pressure);
}

bool sensor_read_imu() {
  if (!sensor_imu_ready) return false;

#if IMU_BNO08XX == 1
  return bno08x_dev->dataAvailable();
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

// =============================================================================
// TEST FUNCTIONS
// =============================================================================

void sensor_test_gps() {
  if (!sensor_gps_ready) {
    LOG_W(LOG_MODULE_GPS, "Not ready");
    return;
  }

  LOG_I(LOG_MODULE_GPS, "Fix: %d | Sats: %d", gps_data.fix, gps_data.satellites);
  if (gps_data.fix) {
    LOG_I(LOG_MODULE_GPS, "Lat: %.6f | Lon: %.6f | Alt: %.0fm",
          gps_data.latitude, gps_data.longitude, gps_data.altitude);
  }
}

void sensor_test_bmp5() {
  if (!sensor_bmp5_ready) {
    LOG_W(LOG_MODULE_BMP5, "Not ready");
    return;
  }

  bmp5_data_t data;
  if (BMP5_read(&bmp5_dev, &data, 1013.25f)) {
    LOG_I(LOG_MODULE_BMP5, "T=%.1f°C P=%.0fhPa Alt=%.0fm",
          data.temperature, data.pressure, data.altitude);
  }
}

void sensor_test_imu() {
  if (!sensor_imu_ready) {
    LOG_W(LOG_MODULE_IMU, "Not ready");
    return;
  }

#if IMU_BNO08XX == 1
  LOG_I(LOG_MODULE_IMU, "BNO08x: call dataAvailable() in main loop");
#else
  lsm6dso32_data_t data;
  if (LSM6DSO32_read(&lsm6dso32_dev, &data)) {
    LOG_I(LOG_MODULE_IMU, "Accel X=%.2f Y=%.2f Z=%.2f", 
          data.accel_x, data.accel_y, data.accel_z);
    LOG_I(LOG_MODULE_IMU, "Gyro X=%.2f Y=%.2f Z=%.2f",
          data.gyro_x, data.gyro_y, data.gyro_z);
  }
#endif
}

void sensor_test_battery() {
  if (!sensor_battery_ready) {
    LOG_W(LOG_MODULE_SYSTEM, "Battery not ready");
    return;
  }

  max17048_data_t data;
  if (MAX17048_read(&max17048_dev, &data)) {
    LOG_I(LOG_MODULE_SYSTEM, "Battery: %.0f%% (%.2fV)", data.soc, data.voltage);
  }
}

// =============================================================================
// INIT ALL & SUMMARY
// =============================================================================

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