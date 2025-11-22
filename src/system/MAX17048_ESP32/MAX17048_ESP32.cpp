/**
 * @file MAX17048_ESP32.cpp
 * @brief Implémentation driver MAX17048 minimaliste
 */

#include "MAX17048_ESP32.h"
#include "src/system/logger/logger.h"
#include <driver/i2c.h>

// Déclaration des fonctions lock/unlock du wrapper
extern bool i2c_lock(i2c_bus_id_t bus, uint32_t timeout_ms);
extern void i2c_unlock(i2c_bus_id_t bus);

// ============================================================================
// I2C DIRECT (avec lock/unlock pour thread-safety)
// ============================================================================
static i2c_port_t get_port(i2c_bus_id_t bus) {
  return (bus == I2C_BUS_1) ? I2C_NUM_1 : I2C_NUM_0;
}

static bool i2c_write_reg16(max17048_device_t* dev, uint8_t reg, uint16_t value) {
  if (!i2c_lock(dev->bus, I2C_TIMEOUT_MS)) {
    LOG_W(LOG_MODULE_SYSTEM, "I2C lock timeout (write)");
    return false;
  }

  uint8_t data[2];
  data[0] = (value >> 8) & 0xFF;  // MSB first
  data[1] = value & 0xFF;         // LSB

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write(cmd, data, 2, true);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(get_port(dev->bus), cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  i2c_unlock(dev->bus);

  return (ret == ESP_OK);
}

static bool i2c_read_reg16(max17048_device_t* dev, uint8_t reg, uint16_t* value) {
  if (!i2c_lock(dev->bus, I2C_TIMEOUT_MS)) {
    LOG_W(LOG_MODULE_SYSTEM, "I2C lock timeout (read)");
    return false;
  }

  uint8_t data[2];

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(get_port(dev->bus), cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  i2c_unlock(dev->bus);

  if (ret == ESP_OK) {
    *value = ((uint16_t)data[0] << 8) | data[1];  // MSB first
    return true;
  }

  return false;
}

// ============================================================================
// INIT
// ============================================================================
bool MAX17048_init(max17048_device_t* dev, const max17048_config_t* config) {
  if (!dev || !config) return false;

  LOG_I(LOG_MODULE_SYSTEM, "Initializing MAX17048...");

  dev->bus = config->bus;
  dev->address = config->address;
  dev->initialized = false;

  // Lire version pour vérifier présence
  uint16_t version;
  if (!i2c_read_reg16(dev, MAX17048_REG_VERSION, &version)) {
    LOG_E(LOG_MODULE_SYSTEM, "Failed to read MAX17048 version");
    return false;
  }

  dev->version = version;

  LOG_I(LOG_MODULE_SYSTEM, "MAX17048 detected (version=0x%04X)", version);

  // Configurer seuil d'alerte si spécifié
  if (config->alert_threshold > 0) {
    if (!MAX17048_set_alert_threshold(dev, config->alert_threshold)) {
      LOG_W(LOG_MODULE_SYSTEM, "Failed to set alert threshold");
    }
  }

  dev->initialized = true;

  LOG_I(LOG_MODULE_SYSTEM, "MAX17048 initialized successfully");

  return true;
}

// ============================================================================
// READ
// ============================================================================
bool MAX17048_read(max17048_device_t* dev, max17048_data_t* data) {
  if (!dev || !dev->initialized || !data) return false;

  uint16_t vcell_raw, soc_raw, config_raw;

  // Lire voltage
  if (!i2c_read_reg16(dev, MAX17048_REG_VCELL, &vcell_raw)) {
    LOG_E(LOG_MODULE_SYSTEM, "Failed to read VCELL");
    return false;
  }

  // Lire SOC
  if (!i2c_read_reg16(dev, MAX17048_REG_SOC, &soc_raw)) {
    LOG_E(LOG_MODULE_SYSTEM, "Failed to read SOC");
    return false;
  }

  // Lire config (pour status alerte)
  if (!i2c_read_reg16(dev, MAX17048_REG_CONFIG, &config_raw)) {
    LOG_E(LOG_MODULE_SYSTEM, "Failed to read CONFIG");
    return false;
  }

  // Convertir voltage : 78.125 µV/cell
  data->voltage = (vcell_raw >> 4) * 0.00125f;  // 1.25 mV per bit

  // Convertir SOC : MSB = %, LSB = 1/256 %
  data->soc = (soc_raw >> 8) + ((soc_raw & 0xFF) / 256.0f);

  // Status alerte (bit 5 de CONFIG)
  data->alert = (config_raw & 0x0020) != 0;

  // Détection charge (heuristique simple)
  static float last_soc = 0.0f;
  static unsigned long last_read_ms = 0;

  unsigned long now = millis();
  if (last_read_ms > 0 && (now - last_read_ms) > 0) {
    float delta_soc = data->soc - last_soc;
    float delta_time_h = (now - last_read_ms) / 3600000.0f;  // ms → heures

    if (delta_time_h > 0) {
      data->charge_rate = delta_soc / delta_time_h;    // %/h
      data->is_charging = (data->charge_rate > 0.1f);  // Seuil 0.1%/h
    }
  }

  last_soc = data->soc;
  last_read_ms = now;

  return true;
}

// ============================================================================
// READ VOLTAGE ONLY
// ============================================================================
float MAX17048_read_voltage(max17048_device_t* dev) {
  if (!dev || !dev->initialized) return 0.0f;

  uint16_t vcell_raw;
  if (!i2c_read_reg16(dev, MAX17048_REG_VCELL, &vcell_raw)) {
    return 0.0f;
  }

  return (vcell_raw >> 4) * 0.00125f;
}

// ============================================================================
// READ SOC ONLY
// ============================================================================
float MAX17048_read_soc(max17048_device_t* dev) {
  if (!dev || !dev->initialized) return 0.0f;

  uint16_t soc_raw;
  if (!i2c_read_reg16(dev, MAX17048_REG_SOC, &soc_raw)) {
    return 0.0f;
  }

  return (soc_raw >> 8) + ((soc_raw & 0xFF) / 256.0f);
}

// ============================================================================
// SET ALERT THRESHOLD
// ============================================================================
bool MAX17048_set_alert_threshold(max17048_device_t* dev, uint8_t threshold) {
  if (!dev || !dev->initialized) return false;

  // Limiter à 1-32%
  if (threshold < 1) threshold = 1;
  if (threshold > 32) threshold = 32;

  // Lire config actuel
  uint16_t config;
  if (!i2c_read_reg16(dev, MAX17048_REG_CONFIG, &config)) {
    return false;
  }

  // Modifier bits 0-4 (ATHD = Alert Threshold)
  // Formule : ATHD = 32 - threshold
  uint8_t athd = 32 - threshold;
  config = (config & 0xFFE0) | (athd & 0x1F);

  // Écrire nouveau config
  if (!i2c_write_reg16(dev, MAX17048_REG_CONFIG, config)) {
    return false;
  }

  LOG_I(LOG_MODULE_SYSTEM, "Alert threshold set to %d%%", threshold);

  return true;
}

// ============================================================================
// RESET
// ============================================================================
bool MAX17048_reset(max17048_device_t* dev) {
  if (!dev) return false;

  LOG_I(LOG_MODULE_SYSTEM, "Resetting MAX17048...");

  if (!i2c_write_reg16(dev, MAX17048_REG_COMMAND, MAX17048_CMD_RESET)) {
    return false;
  }

  delay(10);

  return true;
}

// ============================================================================
// QUICK START
// ============================================================================
bool MAX17048_quick_start(max17048_device_t* dev) {
  if (!dev || !dev->initialized) return false;

  LOG_I(LOG_MODULE_SYSTEM, "Quick start MAX17048...");

  // Lire MODE
  uint16_t mode;
  if (!i2c_read_reg16(dev, MAX17048_REG_MODE, &mode)) {
    return false;
  }

  // Set bit 14 (Quick Start)
  mode |= 0x4000;

  if (!i2c_write_reg16(dev, MAX17048_REG_MODE, mode)) {
    return false;
  }

  delay(175);  // Attendre fin quick start (~175ms)

  LOG_I(LOG_MODULE_SYSTEM, "Quick start complete");

  return true;
}