/**
 * @file BMP5XX_ESP32.cpp
 * @brief Implémentation driver BMP581 minimaliste
 */

#include "BMP5XX_ESP32.h"
#include "src/system/logger/logger.h"
#include <math.h>
#include <driver/i2c.h>

// Déclaration des fonctions lock/unlock du wrapper
extern bool i2c_lock(i2c_bus_id_t bus, uint32_t timeout_ms);
extern void i2c_unlock(i2c_bus_id_t bus);


// ============================================================================
// I2C DIRECT (bypass wrapper bugué)
// ============================================================================
static i2c_port_t get_port(i2c_bus_id_t bus) {
  return (bus == I2C_BUS_1) ? I2C_NUM_1 : I2C_NUM_0;
}

static bool i2c_write_reg(bmp5_device_t* dev, uint8_t reg, uint8_t value) {
  if (!i2c_lock(dev->bus, I2C_TIMEOUT_MS)) {
    return false;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, value, true);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(get_port(dev->bus), cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  i2c_unlock(dev->bus);

  return (ret == ESP_OK);
}

static bool i2c_read_regs(bmp5_device_t* dev, uint8_t reg, uint8_t* data, size_t len) {
  if (!i2c_lock(dev->bus, I2C_TIMEOUT_MS)) {
    return false;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(get_port(dev->bus), cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  i2c_unlock(dev->bus);

  return (ret == ESP_OK);
}

// ============================================================================
// INIT
// ============================================================================
bool BMP5_init(bmp5_device_t* dev, const bmp5_config_t* config) {
  if (!dev || !config) return false;

  LOG_I(LOG_MODULE_BMP5, "Initializing BMP581...");

  dev->bus = config->bus;
  dev->address = config->address;
  dev->initialized = false;

  // Vérifier chip ID
  uint8_t chip_id;
  if (!i2c_read_regs(dev, BMP5_REG_CHIP_ID, &chip_id, 1)) {
    LOG_E(LOG_MODULE_BMP5, "Failed to read chip ID");
    return false;
  }

  if (chip_id != BMP5_CHIPID_581) {
    LOG_E(LOG_MODULE_BMP5, "Invalid chip ID: 0x%02X", chip_id);
    return false;
  }

  LOG_I(LOG_MODULE_BMP5, "BMP581 detected (ID=0x51)");

  // Soft reset
  if (!i2c_write_reg(dev, BMP5_REG_CMD, BMP5_CMD_SOFT_RESET)) {
    LOG_E(LOG_MODULE_BMP5, "Soft reset failed");
    return false;
  }

  delay(10);

  // ✅ Configuration DIRECTE (valeurs testées qui marchent)

  // 1. OSR_CONFIG = 0x61 (OSR_P=16X, OSR_T=2X)
  // Format : bits 6-4 = OSR_P, bits 2-0 = OSR_T
  uint8_t osr_config = ((config->osr_press & 0x07) << 4) | (config->osr_temp & 0x07);

  LOG_I(LOG_MODULE_BMP5, "Writing OSR_CONFIG = 0x%02X", osr_config);
  if (!i2c_write_reg(dev, BMP5_REG_OSR_CONFIG, osr_config)) {
    LOG_E(LOG_MODULE_BMP5, "OSR_CONFIG write failed");
    return false;
  }

  // 2. DSP_IIR (IIR filter pour temp et pression)
  // Format : bits 6-4 = IIR_P, bits 2-0 = IIR_T
  uint8_t dsp_iir = ((config->iir_filter & 0x07) << 4) | (config->iir_filter & 0x07);

  LOG_I(LOG_MODULE_BMP5, "Writing DSP_IIR = 0x%02X", dsp_iir);
  if (!i2c_write_reg(dev, BMP5_REG_DSP_IIR, dsp_iir)) {
    LOG_E(LOG_MODULE_BMP5, "DSP_IIR write failed");
    return false;
  }

  // 3. DSP_CONFIG = 0x2F (format de sortie pression)
  LOG_I(LOG_MODULE_BMP5, "Writing DSP_CONFIG = 0x2F");
  if (!i2c_write_reg(dev, BMP5_REG_DSP_CONFIG, 0x2F)) {
    LOG_E(LOG_MODULE_BMP5, "DSP_CONFIG write failed");
    return false;
  }

  // 4. ODR_CONFIG (mode NORMAL + ODR)
  uint8_t odr_config = 0x80 | (config->odr & 0x1F);  // Bit 7 = deep_disable

  LOG_I(LOG_MODULE_BMP5, "Writing ODR_CONFIG = 0x%02X", odr_config);
  if (!i2c_write_reg(dev, BMP5_REG_ODR_CONFIG, odr_config)) {
    LOG_E(LOG_MODULE_BMP5, "ODR_CONFIG write failed");
    return false;
  }

  delay(100);

  dev->initialized = true;

  LOG_I(LOG_MODULE_BMP5, "BMP581 initialized successfully");

  return true;
}

// ============================================================================
// READ
// ============================================================================
bool BMP5_read(bmp5_device_t* dev, bmp5_data_t* data, float qnh) {
  if (!dev || !dev->initialized || !data) return false;

  // Lire 6 bytes (3 temp + 3 press)
  uint8_t raw[6];
  if (!i2c_read_regs(dev, BMP5_REG_TEMP_XLSB, raw, 6)) {
    LOG_E(LOG_MODULE_BMP5, "Read failed");
    return false;
  }

  // Parser température (24-bit)
  int32_t raw_temp = (int32_t)((uint32_t)raw[2] << 16 | (uint16_t)raw[1] << 8 | raw[0]);
  data->temperature = (float)(raw_temp / 65536.0);

  // Parser pression (24-bit)
  uint32_t raw_press = (uint32_t)((uint32_t)raw[5] << 16 | (uint16_t)raw[4] << 8 | raw[3]);
  data->pressure = (float)(raw_press / 64.0) / 100.0;  // Pa → hPa

  // Calculer altitude
  data->altitude = 44330.0f * (1.0f - powf(data->pressure / qnh, 0.1903f));

  return true;
}

// ============================================================================
// RESET
// ============================================================================
bool BMP5_reset(bmp5_device_t* dev) {
  if (!dev) return false;

  return i2c_write_reg(dev, BMP5_REG_CMD, BMP5_CMD_SOFT_RESET);
}

// ============================================================================
// READ PRESSURE ONLY (optimisation 50% données)
// ============================================================================
bool BMP5_read_pressure_only(bmp5_device_t* dev, float* pressure) {
  if (!dev || !dev->initialized || !pressure) return false;

  // Lire seulement 3 bytes de pression (au lieu de 6)
  uint8_t raw[3];
  if (!i2c_read_regs(dev, BMP5_REG_PRESS_XLSB, raw, 3)) {
    LOG_E(LOG_MODULE_BMP5, "Pressure read failed");
    return false;
  }

  // Parser pression (24-bit, little-endian)
  uint32_t raw_press = (uint32_t)((uint32_t)raw[2] << 16 | (uint16_t)raw[1] << 8 | raw[0]);

  // Convertir en hPa (formule datasheet)
  *pressure = (float)(raw_press / 64.0) / 100.0;  // Pa → hPa

  LOG_V(LOG_MODULE_BMP5, "Pressure only: %.2f hPa", *pressure);

  return true;
}

// ============================================================================
// READ TEMPERATURE ONLY
// ============================================================================
bool BMP5_read_temperature_only(bmp5_device_t* dev, float* temperature) {
  if (!dev || !dev->initialized || !temperature) return false;

  // Lire seulement 3 bytes de température (au lieu de 6)
  uint8_t raw[3];
  if (!i2c_read_regs(dev, BMP5_REG_TEMP_XLSB, raw, 3)) {
    LOG_E(LOG_MODULE_BMP5, "Temperature read failed");
    return false;
  }

  // Parser température (24-bit signed, little-endian)
  int32_t raw_temp = (int32_t)((uint32_t)raw[2] << 16 | (uint16_t)raw[1] << 8 | raw[0]);

  // Convertir en °C (formule datasheet)
  *temperature = (float)(raw_temp / 65536.0);

  LOG_V(LOG_MODULE_BMP5, "Temperature only: %.2f °C", *temperature);

  return true;
}
