/**
 * @file BMP5XX_ESP32.cpp
 * @brief Implémentation driver BMP5XX basé sur Adafruit
 * 
 * @author Franck Moreau (adaptation)
 * @date 2025-11-21
 */

#include "BMP5XX_ESP32.h"
#include "src/system/logger/logger.h"
#include <math.h>

// ============================================================================
// FONCTIONS PRIVÉES
// ============================================================================
static bool bmp5_read_register(bmp5_t* bmp, uint8_t reg, uint8_t* data, uint16_t len);
static bool bmp5_write_register(bmp5_t* bmp, uint8_t reg, uint8_t value);
static bool bmp5_read_coefficients(bmp5_t* bmp);
static void bmp5_quantize_coefficients(bmp5_t* bmp, const bmp5_nvm_par_t* nvm_par);
static float bmp5_compensate_temperature(bmp5_t* bmp, uint32_t raw_temp);
static float bmp5_compensate_pressure(bmp5_t* bmp, uint32_t raw_press, float t_lin);

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
// LECTURE COEFFICIENTS (depuis Adafruit)
// ============================================================================
static bool bmp5_read_coefficients(bmp5_t* bmp) {
    LOG_V(LOG_MODULE_BMP5, "Reading calibration coefficients...");
    
    uint8_t coeffs[21];
    
    // Lire 21 bytes de calibration
    if (!bmp5_read_register(bmp, BMP5_REGISTER_CALIB_DATA, coeffs, 21)) {
        LOG_E(LOG_MODULE_BMP5, "Failed to read calibration");
        return false;
    }
    
    // Parser les coefficients NVM bruts
    bmp5_nvm_par_t nvm_par;
    nvm_par.par_t1 = (uint16_t)(coeffs[0] | (coeffs[1] << 8));
    nvm_par.par_t2 = (uint16_t)(coeffs[2] | (coeffs[3] << 8));
    nvm_par.par_t3 = (int8_t)coeffs[4];
    nvm_par.par_p1 = (int16_t)(coeffs[5] | (coeffs[6] << 8));
    nvm_par.par_p2 = (int16_t)(coeffs[7] | (coeffs[8] << 8));
    nvm_par.par_p3 = (int8_t)coeffs[9];
    nvm_par.par_p4 = (int8_t)coeffs[10];
    nvm_par.par_p5 = (uint16_t)(coeffs[11] | (coeffs[12] << 8));
    nvm_par.par_p6 = (uint16_t)(coeffs[13] | (coeffs[14] << 8));
    nvm_par.par_p7 = (int8_t)coeffs[15];
    nvm_par.par_p8 = (int8_t)coeffs[16];
    nvm_par.par_p9 = (int16_t)(coeffs[17] | (coeffs[18] << 8));
    nvm_par.par_p10 = (int8_t)coeffs[19];
    nvm_par.par_p11 = (int8_t)coeffs[20];
    
    LOG_I(LOG_MODULE_BMP5, "NVM: T1=%u T2=%u T3=%d P1=%d P5=%u", 
          nvm_par.par_t1, nvm_par.par_t2, nvm_par.par_t3, 
          nvm_par.par_p1, nvm_par.par_p5);
    
    // Quantifier les coefficients
    bmp5_quantize_coefficients(bmp, &nvm_par);
    
    LOG_I(LOG_MODULE_BMP5, "Quantized: T1=%.2f T2=%.10f P1=%.6f", 
          bmp->calib.par_t1, bmp->calib.par_t2, bmp->calib.par_p1);
    
    return true;
}

// ============================================================================
// QUANTIFICATION COEFFICIENTS (algorithme Adafruit exact)
// ============================================================================
static void bmp5_quantize_coefficients(bmp5_t* bmp, const bmp5_nvm_par_t* nvm_par) {
    // Température (selon Adafruit_BMP5XX.cpp ligne 183+)
    bmp->calib.par_t1 = (float)nvm_par->par_t1 / powf(2.0f, -8.0f);
    bmp->calib.par_t2 = (float)nvm_par->par_t2 / powf(2.0f, 30.0f);
    bmp->calib.par_t3 = (float)nvm_par->par_t3 / powf(2.0f, 48.0f);
    
    // Pression (selon Adafruit_BMP5XX.cpp ligne 188+)
    bmp->calib.par_p1 = ((float)nvm_par->par_p1 - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
    bmp->calib.par_p2 = ((float)nvm_par->par_p2 - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
    bmp->calib.par_p3 = (float)nvm_par->par_p3 / powf(2.0f, 32.0f);
    bmp->calib.par_p4 = (float)nvm_par->par_p4 / powf(2.0f, 37.0f);
    bmp->calib.par_p5 = (float)nvm_par->par_p5 / powf(2.0f, -3.0f);
    bmp->calib.par_p6 = (float)nvm_par->par_p6 / powf(2.0f, 6.0f);
    bmp->calib.par_p7 = (float)nvm_par->par_p7 / powf(2.0f, 8.0f);
    bmp->calib.par_p8 = (float)nvm_par->par_p8 / powf(2.0f, 15.0f);
    bmp->calib.par_p9 = (float)nvm_par->par_p9 / powf(2.0f, 48.0f);
    bmp->calib.par_p10 = (float)nvm_par->par_p10 / powf(2.0f, 48.0f);
    bmp->calib.par_p11 = (float)nvm_par->par_p11 / powf(2.0f, 65.0f);
}

// ============================================================================
// COMPENSATION TEMPÉRATURE (algorithme Adafruit exact)
// ============================================================================
static float bmp5_compensate_temperature(bmp5_t* bmp, uint32_t raw_temp) {
    // Selon Adafruit_BMP5XX.cpp ligne 213+
    float partial_data1 = (float)raw_temp - bmp->calib.par_t1;
    float partial_data2 = partial_data1 * bmp->calib.par_t2;
    
    float t_lin = partial_data2 + (partial_data1 * partial_data1) * bmp->calib.par_t3;
    
    return t_lin;
}

// ============================================================================
// COMPENSATION PRESSION (algorithme Adafruit exact)
// ============================================================================
static float bmp5_compensate_pressure(bmp5_t* bmp, uint32_t raw_press, float t_lin) {
    // Selon Adafruit_BMP5XX.cpp ligne 228+
    float partial_data1 = bmp->calib.par_p6 * t_lin;
    float partial_data2 = bmp->calib.par_p7 * (t_lin * t_lin);
    float partial_data3 = bmp->calib.par_p8 * (t_lin * t_lin * t_lin);
    float partial_out1 = bmp->calib.par_p5 + partial_data1 + partial_data2 + partial_data3;
    
    partial_data1 = bmp->calib.par_p2 * t_lin;
    partial_data2 = bmp->calib.par_p3 * (t_lin * t_lin);
    partial_data3 = bmp->calib.par_p4 * (t_lin * t_lin * t_lin);
    float partial_out2 = (float)raw_press * 
                         (bmp->calib.par_p1 + partial_data1 + partial_data2 + partial_data3);
    
    partial_data1 = (float)raw_press * (float)raw_press;
    partial_data2 = bmp->calib.par_p9 + bmp->calib.par_p10 * t_lin;
    partial_data3 = partial_data1 * partial_data2;
    float partial_data4 = partial_data3 + 
                          ((float)raw_press * (float)raw_press * (float)raw_press) * 
                          bmp->calib.par_p11;
    
    return partial_out1 + partial_out2 + partial_data4;
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
          bmp->chip_id == BMP5_CHIPID_581 ? "BMP581" : "BMP5", bmp->chip_id);
    
    // Soft reset
    if (!BMP5_reset(bmp)) {
        LOG_E(LOG_MODULE_BMP5, "Reset failed");
        return false;
    }
    
    delay(10);
    
    // Lire coefficients
    if (!bmp5_read_coefficients(bmp)) {
        return false;
    }
    
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
    
    // Configurer ODR
    if (!bmp5_write_register(bmp, BMP5_REGISTER_ODR_CONFIG, (uint8_t)bmp->odr)) {
        LOG_E(LOG_MODULE_BMP5, "Failed to set ODR");
        return false;
    }
    
    // Démarrer en mode continu
    if (!BMP5_set_mode(bmp, bmp->mode)) {
        LOG_E(LOG_MODULE_BMP5, "Failed to set mode");
        return false;
    }
    
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
// LECTURE DONNÉES
// ============================================================================
bool BMP5_read(bmp5_t* bmp) {
    if (!bmp || !bmp->initialized) {
        LOG_E(LOG_MODULE_BMP5, "Not initialized");
        return false;
    }
    
    uint8_t data[6];
    
    // Lire température (3 bytes) + pression (3 bytes)
    if (!bmp5_read_register(bmp, BMP5_REGISTER_TEMP_DATA_XLSB, data, 6)) {
        LOG_E(LOG_MODULE_BMP5, "Failed to read sensor data");
        return false;
    }
    
    // Parser température (24 bits, little-endian)
    uint32_t raw_temp = (uint32_t)(data[0] | (data[1] << 8) | (data[2] << 16));
    
    // Parser pression (24 bits, little-endian)
    uint32_t raw_press = (uint32_t)(data[3] | (data[4] << 8) | (data[5] << 16));
    
    LOG_V(LOG_MODULE_BMP5, "Raw: T=%u P=%u", raw_temp, raw_press);
    
    // Compenser température
    float t_lin = bmp5_compensate_temperature(bmp, raw_temp);
    bmp->temperature = t_lin;
    
    // Compenser pression
    bmp->pressure = bmp5_compensate_pressure(bmp, raw_press, t_lin);
    
    bmp->last_read_ms = millis();
    
    LOG_V(LOG_MODULE_BMP5, "Compensated: %.2f°C %.2fhPa", 
          bmp->temperature, bmp->pressure / 100.0f);
    
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
    
    // Modifier bits 0-1 pour le mode
    osr_config = (osr_config & 0xFC) | ((uint8_t)mode & 0x03);
    
    if (!bmp5_write_register(bmp, BMP5_REGISTER_OSR_CONFIG, osr_config)) {
        return false;
    }
    
    bmp->mode = mode;
    LOG_V(LOG_MODULE_BMP5, "Mode set to %d", mode);
    
    return true;
}