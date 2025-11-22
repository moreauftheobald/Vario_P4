/**
 * @file BMP5XX_ESP32.cpp
 * @brief Driver BMP5XX simplifié - Lecture directe des données compensées
 */

#include "BMP5XX_ESP32.h"
#include "src/system/logger/logger.h"
#include <math.h>

// ============================================================================
// FONCTIONS PRIVÉES
// ============================================================================
static bool bmp5_read_register(bmp5_t* bmp, uint8_t reg, uint8_t* data, uint16_t len);
static bool bmp5_write_register(bmp5_t* bmp, uint8_t reg, uint8_t value);

// ============================================================================
// I2C WRAPPER
// ============================================================================
static bool bmp5_read_register(bmp5_t* bmp, uint8_t reg, uint8_t* data, uint16_t len) {
  return i2c_read_bytes(bmp->bus, bmp->address, reg, data, len);
}

static bool bmp5_write_register(bmp5_t* bmp, uint8_t reg, uint8_t value) {
  return i2c_write_bytes(bmp->bus, bmp->address, reg, &value, 1);
}

// ============================================================================
// INITIALISATION
// ============================================================================
bool BMP5_init(bmp5_t* bmp, const bmp5_config_t* config) {
  if (!bmp || !config) return false;

  LOG_I(LOG_MODULE_BMP5, "Initializing BMP5XX...");

  bmp->bus = config->bus;
  bmp->address = config->address;
  bmp->osr_temp = config->osr_temp;
  bmp->osr_press = config->osr_press;
  bmp->odr = config->odr;
  bmp->iir_filter = config->iir_filter;
  bmp->mode = config->mode;
  bmp->initialized = false;

  // Vérifier présence
  if (!i2c_probe_device(bmp->bus, bmp->address)) {
    LOG_E(LOG_MODULE_BMP5, "Sensor not responding at 0x%02X", bmp->address);
    return false;
  }

  // Lire chip ID
  if (!bmp5_read_register(bmp, BMP5_REGISTER_CHIPID, &bmp->chip_id, 1)) {
    LOG_E(LOG_MODULE_BMP5, "Failed to read chip ID");
    return false;
  }

  if (bmp->chip_id != BMP5_CHIPID_585 && bmp->chip_id != BMP5_CHIPID_581) {
    LOG_E(LOG_MODULE_BMP5, "Invalid chip ID: 0x%02X", bmp->chip_id);
    return false;
  }

  LOG_I(LOG_MODULE_BMP5, "Detected %s (ID=0x%02X)",
        bmp->chip_id == BMP5_CHIPID_581 ? "BMP581" : "BMP585", bmp->chip_id);

  // Soft reset
  if (!BMP5_reset(bmp)) {
    LOG_E(LOG_MODULE_BMP5, "Reset failed");
    return false;
  }

  delay(50);

  // Configurer OSR
  uint8_t osr_config = ((uint8_t)bmp->osr_press << 3) | (uint8_t)bmp->osr_temp;
  if (!bmp5_write_register(bmp, BMP5_REGISTER_OSR_CONFIG, osr_config)) {
    LOG_E(LOG_MODULE_BMP5, "Failed to set OSR");
    return false;
  }

  // Configurer IIR
  if (!bmp5_write_register(bmp, BMP5_REGISTER_DSP_IIR, (uint8_t)bmp->iir_filter)) {
    LOG_E(LOG_MODULE_BMP5, "Failed to set IIR");
    return false;
  }

  // ✅ AJOUT : Forcer le format de sortie pression en mode "Normal"
  // Registre DSP_CONFIG (0x30) : bit 2 = 0 pour format standard
  uint8_t dsp_config = 0x00;  // Tous les bits à 0 = format standard
  if (!bmp5_write_register(bmp, BMP5_REGISTER_DSP_CONFIG, dsp_config)) {
    LOG_E(LOG_MODULE_BMP5, "Failed to set DSP config");
    return false;
  }

  // Configurer ODR
  LOG_V(LOG_MODULE_BMP5, "Setting ODR...");

  // ✅ CORRECTION : S'assurer que deep_disable = 1 (bit 7)
  uint8_t odr_value = (uint8_t)bmp->odr | 0x80;  // Forcer bit 7 = 1 (deep standby disabled)

  if (!bmp5_write_register(bmp, BMP5_REGISTER_ODR_CONFIG, odr_value)) {
    LOG_E(LOG_MODULE_BMP5, "Failed to set ODR");
    return false;
  }

  // Vérifier
  uint8_t odr_verify;
  if (!bmp5_read_register(bmp, BMP5_REGISTER_ODR_CONFIG, &odr_verify, 1)) {
    LOG_E(LOG_MODULE_BMP5, "Failed to verify ODR");
    return false;
  }

  LOG_I(LOG_MODULE_BMP5, "ODR config: wrote=0x%02X read=0x%02X", odr_value, odr_verify);

  // ✅ AJOUT : Clear le bit POR (Power-On Reset)
  LOG_V(LOG_MODULE_BMP5, "Clearing POR flag...");

  // Lire le statut actuel
  uint8_t status;
  if (!bmp5_read_register(bmp, BMP5_REGISTER_STATUS, &status, 1)) {
    LOG_E(LOG_MODULE_BMP5, "Failed to read status");
    return false;
  }

  LOG_V(LOG_MODULE_BMP5, "Status before clear: 0x%02X", status);

  // Clear le bit POR en écrivant 1 sur le bit 0
  if (status & 0x01) {
    if (!bmp5_write_register(bmp, BMP5_REGISTER_STATUS, 0x01)) {
      LOG_E(LOG_MODULE_BMP5, "Failed to clear POR flag");
      return false;
    }

    delay(10);

    // Vérifier que le bit est bien cleared
    if (!bmp5_read_register(bmp, BMP5_REGISTER_STATUS, &status, 1)) {
      LOG_E(LOG_MODULE_BMP5, "Failed to verify status");
      return false;
    }

    LOG_I(LOG_MODULE_BMP5, "Status after clear: 0x%02X", status);
  }

  // Démarrer en mode continu
  LOG_V(LOG_MODULE_BMP5, "Starting continuous mode...");

  // ✅ AJOUT : Passer d'abord en FORCED pour "réveiller" le capteur
  if (!BMP5_set_mode(bmp, BMP5_MODE_FORCED)) {
    LOG_E(LOG_MODULE_BMP5, "Failed to set FORCED mode");
    return false;
  }

  delay(50);  // Attendre une mesure

  // Maintenant passer en CONTINUOUS
  if (!BMP5_set_mode(bmp, BMP5_MODE_CONTINUOUS)) {
    LOG_E(LOG_MODULE_BMP5, "Failed to set CONTINUOUS mode");
    return false;
  }

  delay(50);  // Laisser le temps au capteur de démarrer

  // ✅ AJOUT TEMPORAIRE : Dump tous les registres
  LOG_I(LOG_MODULE_BMP5, "=== Register Dump ===");

  uint8_t regs[] = {
    BMP5_REGISTER_CHIPID,
    BMP5_REGISTER_STATUS,
    BMP5_REGISTER_OSR_CONFIG,
    BMP5_REGISTER_ODR_CONFIG,
    BMP5_REGISTER_DSP_CONFIG,
    BMP5_REGISTER_DSP_IIR
  };

  const char* names[] = {
    "CHIPID",
    "STATUS",
    "OSR_CONFIG",
    "ODR_CONFIG",
    "DSP_CONFIG",
    "DSP_IIR"
  };

  for (int i = 0; i < 6; i++) {
    uint8_t val;
    if (bmp5_read_register(bmp, regs[i], &val, 1)) {
      LOG_I(LOG_MODULE_BMP5, "  0x%02X %-12s = 0x%02X", regs[i], names[i], val);
    }
  }

  LOG_I(LOG_MODULE_BMP5, "====================");

  bmp->initialized = true;
  bmp->last_read_ms = millis();

  LOG_I(LOG_MODULE_BMP5, "BMP5XX initialized @ 0x%02X", bmp->address);

  return true;
}

// ============================================================================
// RESET
// ============================================================================
bool BMP5_reset(bmp5_t* bmp) {
  LOG_V(LOG_MODULE_BMP5, "Soft reset...");
  return bmp5_write_register(bmp, BMP5_REGISTER_CMD, BMP5_CMD_SOFTRESET);
}

// ============================================================================
// LECTURE DONNÉES (déjà compensées par le capteur)
// ============================================================================
bool BMP5_read(bmp5_t* bmp) {
  if (!bmp || !bmp->initialized) {
    LOG_E(LOG_MODULE_BMP5, "Not initialized");
    return false;
  }

  // ✅ AJOUT : Vérifier si les données sont prêtes
  uint8_t status;
  if (!bmp5_read_register(bmp, BMP5_REGISTER_STATUS, &status, 1)) {
    LOG_E(LOG_MODULE_BMP5, "Failed to read status");
    return false;
  }

  // Bit 5 = drdy_press, Bit 6 = drdy_temp
  if (!(status & 0x60)) {  // Les deux bits doivent être à 1
    LOG_V(LOG_MODULE_BMP5, "Data not ready (status=0x%02X)", status);
    return false;  // Données pas encore prêtes
  }

  uint8_t data[6];

  // Lire température (3 bytes) + pression (3 bytes)
  if (!bmp5_read_register(bmp, BMP5_REGISTER_TEMP_DATA_XLSB, data, 6)) {
    LOG_E(LOG_MODULE_BMP5, "Failed to read sensor data");
    return false;
  }

  // Parser température (24 bits, little-endian, SIGNÉ)
  int32_t raw_temp = (int32_t)(data[0] | (data[1] << 8) | (data[2] << 16));
  // Étendre le signe si bit 23 = 1
  if (raw_temp & 0x800000) {
    raw_temp |= 0xFF000000;
  }

  // Parser pression (24 bits, little-endian, SIGNÉ)
  int32_t raw_press = (int32_t)(data[3] | (data[4] << 8) | (data[5] << 16));
  // Étendre le signe si bit 23 = 1
  if (raw_press & 0x800000) {
    raw_press |= 0xFF000000;
  }

  LOG_V(LOG_MODULE_BMP5, "Raw: T=%d P=%d", raw_temp, raw_press);

  // Conversion en unités physiques (valeurs déjà compensées par le capteur)
  // Selon datasheet BMP581 section 4.3.9:
  // - Température : LSB = 2^-16 °C
  // - Pression : LSB = 2^-6 Pa (soit 1/64 Pa)

  bmp->temperature = (float)raw_temp / 65536.0f;  // Division par 2^16
  bmp->pressure = (float)raw_press / 256.0f;      // Division par 2^8

  bmp->last_read_ms = millis();

  LOG_V(LOG_MODULE_BMP5, "Calibrated: %.2f°C %.2fPa",
        bmp->temperature, bmp->pressure);

  return true;
}

// ============================================================================
// GETTERS
// ============================================================================
float BMP5_get_temperature(const bmp5_t* bmp) {
  return bmp ? bmp->temperature : 0.0f;
}

float BMP5_get_pressure(const bmp5_t* bmp) {
  return bmp ? bmp->pressure : 0.0f;
}

float BMP5_get_pressure_hPa(const bmp5_t* bmp) {
  return bmp ? (bmp->pressure / 100.0f) : 0.0f;
}

float BMP5_calculate_altitude(const bmp5_t* bmp, float qnh) {
  if (!bmp) return 0.0f;

  float pressure_hPa = bmp->pressure / 100.0f;
  float altitude = 44330.0f * (1.0f - powf(pressure_hPa / qnh, 0.1903f));

  return altitude;
}

// ============================================================================
// CHANGEMENT MODE
// ============================================================================
bool BMP5_set_mode(bmp5_t* bmp, bmp5_powermode_t mode) {
  if (!bmp) return false;

  uint8_t osr_config;
  if (!bmp5_read_register(bmp, BMP5_REGISTER_OSR_CONFIG, &osr_config, 1)) {
    return false;
  }

  LOG_V(LOG_MODULE_BMP5, "Current OSR_CONFIG: 0x%02X", osr_config);

  // Modifier bits 0-1 pour le mode
  osr_config = (osr_config & 0xFC) | ((uint8_t)mode & 0x03);

  LOG_V(LOG_MODULE_BMP5, "New OSR_CONFIG: 0x%02X (mode=%d)", osr_config, mode);

  if (!bmp5_write_register(bmp, BMP5_REGISTER_OSR_CONFIG, osr_config)) {
    return false;
  }

  // ✅ AJOUT : Vérifier que l'écriture a bien fonctionné
  uint8_t verify;
  if (!bmp5_read_register(bmp, BMP5_REGISTER_OSR_CONFIG, &verify, 1)) {
    return false;
  }

  LOG_I(LOG_MODULE_BMP5, "Mode set: wrote=0x%02X read=0x%02X", osr_config, verify);

  if (verify != osr_config) {
    LOG_E(LOG_MODULE_BMP5, "Mode verification failed!");
    return false;
  }

  bmp->mode = mode;

  return true;
}