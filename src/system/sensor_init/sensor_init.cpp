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
gps_i2c_esp32_t gps;
bool sensor_gps_ready = false;

bmp5_device_t bmp5_dev;
bool sensor_bmp5_ready = false;

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

  // Configuration GPS I2C
  gps_i2c_esp32_config_t config = {
    .bus = I2C_BUS_1,
    .i2c_addr = GPS_I2C_ADDR
  };

  // Initialiser la structure GPS
  esp_err_t ret = GPS_I2C_ESP32_init(&gps, &config);
  if (ret != ESP_OK) {
    LOG_E(LOG_MODULE_GPS, "GPS init failed: %s", esp_err_to_name(ret));
    sensor_gps_ready = false;
    return false;
  }

  LOG_V(LOG_MODULE_GPS, "GPS structure initialized");

  // Vérifier présence du GPS sur le bus I2C
  if (!i2c_probe_device(I2C_BUS_1, GPS_I2C_ADDR)) {
    LOG_E(LOG_MODULE_GPS, "GPS not responding at 0x%02X", GPS_I2C_ADDR);
    sensor_gps_ready = false;
    return false;
  }

  LOG_V(LOG_MODULE_GPS, "GPS responding on I2C");

  // Envoyer commande de réveil (au cas où)
  delay(100);
  ret = GPS_I2C_ESP32_send_command(&gps, "");
  if (ret != ESP_OK) {
    LOG_W(LOG_MODULE_GPS, "Wake command warning (may be normal): %s",
          esp_err_to_name(ret));
  }

  delay(100);

  // Configuration GPS : sortie RMC + GGA
  LOG_V(LOG_MODULE_GPS, "Configuring GPS output (RMC+GGA)...");
  ret = GPS_I2C_ESP32_send_command(&gps, PMTK_SET_NMEA_OUTPUT_RMCGGA);
  if (ret != ESP_OK) {
    LOG_W(LOG_MODULE_GPS, "Output config warning: %s", esp_err_to_name(ret));
  }

  delay(100);

  // Configuration GPS : taux de mise à jour 1Hz
  LOG_V(LOG_MODULE_GPS, "Setting GPS update rate to 1Hz...");
  ret = GPS_I2C_ESP32_send_command(&gps, PMTK_SET_NMEA_UPDATE_1HZ);
  if (ret != ESP_OK) {
    LOG_W(LOG_MODULE_GPS, "Update rate warning: %s", esp_err_to_name(ret));
  }

  delay(100);

  // Configuration GPS : fix à 1Hz
  LOG_V(LOG_MODULE_GPS, "Setting GPS fix rate to 1Hz...");
  ret = GPS_I2C_ESP32_send_command(&gps, PMTK_API_SET_FIX_CTL_1HZ);
  if (ret != ESP_OK) {
    LOG_W(LOG_MODULE_GPS, "Fix rate warning: %s", esp_err_to_name(ret));
  }

  LOG_I(LOG_MODULE_GPS, "GPS PA1010D initialized @ 1Hz (RMC+GGA)");
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
// === LECTURE GPS (à appeler cycliquement)
// ================================
bool sensor_read_gps() {
  if (!sensor_gps_ready) {
    return false;
  }

  // Lire les données disponibles
  char c = GPS_I2C_ESP32_read(&gps);

  // Si on a reçu une ligne complète
  if (c == '\n') {
    // Récupérer la dernière ligne NMEA
    char* nmea = gps.lastline;

    // Parser la ligne
    if (GPS_I2C_ESP32_parse(&gps, nmea)) {
      LOG_V(LOG_MODULE_GPS, "NMEA parsed: %s", nmea);

      // Afficher infos si fix obtenu
      if (gps.fix) {
        LOG_V(LOG_MODULE_GPS,
              "Fix: %d sats, Lat: %.6f, Lon: %.6f, Alt: %.1f m",
              gps.satellites,
              gps.latitudeDegrees,
              gps.longitudeDegrees,
              gps.altitude);
      }

      return true;
    } else {
      LOG_V(LOG_MODULE_GPS, "NMEA parse failed or unknown sentence");
    }
  }

  return false;
}

// ================================
// === LECTURE BMP5 (à appeler cycliquement)
// ================================
bool sensor_read_bmp5() {
  if (!sensor_bmp5_ready) return false;

  bmp5_data_t data;
  if (BMP5_read(&bmp5_dev, &data, 1013.25f)) {
    LOG_I(LOG_MODULE_BMP5, "T=%.2f°C P=%.2fhPa Alt=%.1fm",
          data.temperature, data.pressure, data.altitude);
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
  LOG_I(LOG_MODULE_GPS, "Fix: %s", gps.fix ? "YES" : "NO");
  LOG_I(LOG_MODULE_GPS, "Satellites: %d", gps.satellites);
  LOG_I(LOG_MODULE_GPS, "Fix Quality: %d", gps.fixquality);

  if (gps.fix) {
    LOG_I(LOG_MODULE_GPS, "Position: %.6f, %.6f",
          gps.latitudeDegrees, gps.longitudeDegrees);
    LOG_I(LOG_MODULE_GPS, "Altitude: %.1f m", gps.altitude);
    LOG_I(LOG_MODULE_GPS, "Speed: %.1f knots", gps.speed);
    LOG_I(LOG_MODULE_GPS, "HDOP: %.1f", gps.HDOP);

    // Temps
    LOG_I(LOG_MODULE_GPS, "Time: %02d:%02d:%02d.%03d UTC",
          gps.hour, gps.minute, gps.seconds, gps.milliseconds);
    LOG_I(LOG_MODULE_GPS, "Date: %02d/%02d/20%02d",
          gps.day, gps.month, gps.year);
  } else {
    LOG_I(LOG_MODULE_GPS, "Waiting for GPS fix...");
    float elapsed = GPS_I2C_ESP32_seconds_since_fix(&gps);
    if (elapsed < 1000000) {  // Valeur raisonnable
      LOG_I(LOG_MODULE_GPS, "Time since last fix: %.1f s", elapsed);
    }
  }

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

  // Initialiser GPS
  bool gps_ok = sensor_init_gps();

  LOG_I(LOG_MODULE_SYSTEM, "");

  // Initialiser BMP5
  bool BMP5_ok = sensor_init_bmp5();

  LOG_I(LOG_MODULE_SYSTEM, "");

  // Afficher résumé
  sensor_init_print_summary();

  LOG_I(LOG_MODULE_SYSTEM, "=== Init Complete ===");
  LOG_I(LOG_MODULE_SYSTEM, "");

  return gps_ok || BMP5_ok;  // Au moins un capteur OK
}

// ================================
// === SUMMARY
// ================================
void sensor_init_print_summary() {
  LOG_I(LOG_MODULE_SYSTEM, "");
  LOG_I(LOG_MODULE_SYSTEM, "--- Sensors Summary ---");
  LOG_I(LOG_MODULE_SYSTEM, "GPS PA1010D : %s",
        sensor_gps_ready ? "OK" : "FAIL");
  LOG_I(LOG_MODULE_SYSTEM, "BMP5      : %s",
        sensor_bmp5_ready ? "OK" : "FAIL");
  LOG_I(LOG_MODULE_SYSTEM, "LSM6DSO32   : Not initialized (future)");
  LOG_I(LOG_MODULE_SYSTEM, "----------------------");
  LOG_I(LOG_MODULE_SYSTEM, "");
}

void dump_all_bmp5_registers() {
  LOG_I(LOG_MODULE_BMP5, "");
  LOG_I(LOG_MODULE_BMP5, "=== COMPLETE BMP5 REGISTER DUMP ===");

  // Registres importants à vérifier
  struct {
    uint8_t addr;
    const char* name;
  } regs[] = {
    { 0x01, "CHIP_ID" },
    { 0x02, "REV_ID" },
    { 0x11, "CHIP_STATUS" },
    { 0x13, "DRIVE_CONFIG" },
    { 0x14, "INT_CONFIG" },
    { 0x15, "INT_SOURCE" },
    { 0x16, "FIFO_CONFIG" },
    { 0x17, "FIFO_COUNT" },
    { 0x18, "FIFO_SEL" },
    { 0x27, "INT_STATUS" },
    { 0x28, "STATUS" },
    { 0x30, "DSP_CONFIG" },
    { 0x31, "DSP_IIR" },
    { 0x32, "OOR_THR_P_LSB" },
    { 0x33, "OOR_THR_P_MSB" },
    { 0x34, "OOR_RANGE" },
    { 0x35, "OOR_CONFIG" },
    { 0x36, "OSR_CONFIG" },
    { 0x37, "ODR_CONFIG" },
    { 0x38, "OSR_EFF" },
  };

  for (int i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
    uint8_t val;
    if (i2c_read_bytes(I2C_BUS_1, BMP5_I2C_ADDR, regs[i].addr, &val, 1)) {
      LOG_I(LOG_MODULE_BMP5, "  0x%02X %-16s = 0x%02X",
            regs[i].addr, regs[i].name, val);
    } else {
      LOG_E(LOG_MODULE_BMP5, "  0x%02X %-16s = READ FAILED",
            regs[i].addr, regs[i].name);
    }
  }

  // Lire aussi les données brutes
  LOG_I(LOG_MODULE_BMP5, "");
  LOG_I(LOG_MODULE_BMP5, "Data registers (0x1D-0x22):");

  for (uint8_t reg = 0x1D; reg <= 0x22; reg++) {
    uint8_t val;
    if (i2c_read_bytes(I2C_BUS_1, BMP5_I2C_ADDR, reg, &val, 1)) {
      LOG_I(LOG_MODULE_BMP5, "  0x%02X = 0x%02X", reg, val);
    }
  }

  LOG_I(LOG_MODULE_BMP5, "====================================");
  LOG_I(LOG_MODULE_BMP5, "");
}