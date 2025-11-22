/**
 * @file LSM6DSO32_ESP32.cpp
 * @brief Implémentation driver LSM6DSO32 minimaliste
 */

#include "LSM6DSO32_ESP32.h"
#include "src/system/logger/logger.h"
#include <driver/i2c.h>
#include <math.h>

// ============================================================================
// CONSTANTES
// ============================================================================
#define GRAVITY_EARTH 9.80665f  // m/s²
#define DEG_TO_RAD 0.017453292519943295f

// Facteurs de conversion accel (LSB → G)
static const float ACCEL_SCALE_4G = 0.000122f;   // 4G
static const float ACCEL_SCALE_8G = 0.000244f;   // 8G
static const float ACCEL_SCALE_16G = 0.000488f;  // 16G
static const float ACCEL_SCALE_32G = 0.000976f;  // 32G

// Facteurs de conversion gyro (LSB → dps)
static const float GYRO_SCALE_125DPS = 0.004375f;   // 125 dps
static const float GYRO_SCALE_250DPS = 0.00875f;    // 250 dps
static const float GYRO_SCALE_500DPS = 0.0175f;     // 500 dps
static const float GYRO_SCALE_1000DPS = 0.035f;     // 1000 dps
static const float GYRO_SCALE_2000DPS = 0.070f;     // 2000 dps

// ============================================================================
// I2C DIRECT
// ============================================================================
static i2c_port_t get_port(i2c_bus_id_t bus) {
    return (bus == I2C_BUS_1) ? I2C_NUM_1 : I2C_NUM_0;
}

static bool i2c_write_reg(lsm6dso32_device_t* dev, uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(get_port(dev->bus), cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

static bool i2c_read_regs(lsm6dso32_device_t* dev, uint8_t reg, uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(get_port(dev->bus), cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return (ret == ESP_OK);
}

// ============================================================================
// INIT
// ============================================================================
bool LSM6DSO32_init(lsm6dso32_device_t* dev, const lsm6dso32_config_t* config) {
    if (!dev || !config) return false;
    
    LOG_I(LOG_MODULE_IMU, "Initializing LSM6DSO32...");
    
    dev->bus = config->bus;
    dev->address = config->address;
    dev->initialized = false;
    
    // Vérifier WHO_AM_I
    uint8_t chip_id;
    if (!i2c_read_regs(dev, LSM6DSO32_REG_WHO_AM_I, &chip_id, 1)) {
        LOG_E(LOG_MODULE_IMU, "Failed to read WHO_AM_I");
        return false;
    }
    
    if (chip_id != LSM6DSO32_CHIP_ID) {
        LOG_E(LOG_MODULE_IMU, "Invalid chip ID: 0x%02X (expected 0x6C)", chip_id);
        return false;
    }
    
    LOG_I(LOG_MODULE_IMU, "LSM6DSO32 detected (ID=0x6C)");
    
    // Soft reset
    if (!i2c_write_reg(dev, LSM6DSO32_REG_CTRL3_C, 0x01)) {
        LOG_E(LOG_MODULE_IMU, "Soft reset failed");
        return false;
    }
    
    delay(10);
    
    // ✅ Configuration accéléromètre
    // CTRL1_XL: bits 7-4 = ODR, bits 3-2 = range, bits 1-0 = bw
    uint8_t ctrl1_xl = (config->accel_rate << 4) | (config->accel_range << 2);
    
    LOG_I(LOG_MODULE_IMU, "Writing CTRL1_XL = 0x%02X", ctrl1_xl);
    if (!i2c_write_reg(dev, LSM6DSO32_REG_CTRL1_XL, ctrl1_xl)) {
        LOG_E(LOG_MODULE_IMU, "CTRL1_XL write failed");
        return false;
    }
    
    // ✅ Configuration gyroscope
    // CTRL2_G: bits 7-4 = ODR, bits 3-1 = range
    uint8_t ctrl2_g = (config->gyro_rate << 4) | (config->gyro_range << 1);
    
    LOG_I(LOG_MODULE_IMU, "Writing CTRL2_G = 0x%02X", ctrl2_g);
    if (!i2c_write_reg(dev, LSM6DSO32_REG_CTRL2_G, ctrl2_g)) {
        LOG_E(LOG_MODULE_IMU, "CTRL2_G write failed");
        return false;
    }
    
    // ✅ Activer auto-increment (CTRL3_C bit 2)
    if (!i2c_write_reg(dev, LSM6DSO32_REG_CTRL3_C, 0x04)) {
        LOG_E(LOG_MODULE_IMU, "CTRL3_C write failed");
        return false;
    }
    
    // ✅ High performance mode accel (CTRL6_C = 0x00)
    if (!i2c_write_reg(dev, LSM6DSO32_REG_CTRL6_C, 0x00)) {
        LOG_E(LOG_MODULE_IMU, "CTRL6_C write failed");
        return false;
    }
    
    // ✅ High performance mode gyro (CTRL7_G = 0x00)
    if (!i2c_write_reg(dev, LSM6DSO32_REG_CTRL7_G, 0x00)) {
        LOG_E(LOG_MODULE_IMU, "CTRL7_G write failed");
        return false;
    }
    
    // ✅ Calculer les facteurs de conversion
    switch (config->accel_range) {
        case LSM6DSO32_ACCEL_RANGE_4G:  dev->accel_scale = ACCEL_SCALE_4G * GRAVITY_EARTH; break;
        case LSM6DSO32_ACCEL_RANGE_8G:  dev->accel_scale = ACCEL_SCALE_8G * GRAVITY_EARTH; break;
        case LSM6DSO32_ACCEL_RANGE_16G: dev->accel_scale = ACCEL_SCALE_16G * GRAVITY_EARTH; break;
        case LSM6DSO32_ACCEL_RANGE_32G: dev->accel_scale = ACCEL_SCALE_32G * GRAVITY_EARTH; break;
    }
    
    switch (config->gyro_range) {
        case LSM6DSO32_GYRO_RANGE_125_DPS:  dev->gyro_scale = GYRO_SCALE_125DPS * DEG_TO_RAD; break;
        case LSM6DSO32_GYRO_RANGE_250_DPS:  dev->gyro_scale = GYRO_SCALE_250DPS * DEG_TO_RAD; break;
        case LSM6DSO32_GYRO_RANGE_500_DPS:  dev->gyro_scale = GYRO_SCALE_500DPS * DEG_TO_RAD; break;
        case LSM6DSO32_GYRO_RANGE_1000_DPS: dev->gyro_scale = GYRO_SCALE_1000DPS * DEG_TO_RAD; break;
        case LSM6DSO32_GYRO_RANGE_2000_DPS: dev->gyro_scale = GYRO_SCALE_2000DPS * DEG_TO_RAD; break;
    }
    
    delay(20);
    
    dev->initialized = true;
    
    LOG_I(LOG_MODULE_IMU, "LSM6DSO32 initialized successfully");
    
    return true;
}

// ============================================================================
// READ
// ============================================================================
bool LSM6DSO32_read(lsm6dso32_device_t* dev, lsm6dso32_data_t* data) {
    if (!dev || !dev->initialized || !data) return false;
    
    // Lire gyro (6 bytes) + accel (6 bytes) = 12 bytes
    uint8_t raw[12];
    
    // Lire gyro
    if (!i2c_read_regs(dev, LSM6DSO32_REG_OUTX_L_G, raw, 6)) {
        LOG_E(LOG_MODULE_IMU, "Gyro read failed");
        return false;
    }
    
    // Lire accel
    if (!i2c_read_regs(dev, LSM6DSO32_REG_OUTX_L_A, raw + 6, 6)) {
        LOG_E(LOG_MODULE_IMU, "Accel read failed");
        return false;
    }
    
    // Parser gyro (16-bit signed, little-endian)
    int16_t gyro_raw_x = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
    int16_t gyro_raw_y = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
    int16_t gyro_raw_z = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);
    
    // Parser accel (16-bit signed, little-endian)
    int16_t accel_raw_x = (int16_t)((uint16_t)raw[7] << 8 | raw[6]);
    int16_t accel_raw_y = (int16_t)((uint16_t)raw[9] << 8 | raw[8]);
    int16_t accel_raw_z = (int16_t)((uint16_t)raw[11] << 8 | raw[10]);
    
    // Convertir en unités physiques
    data->gyro_x = gyro_raw_x * dev->gyro_scale;
    data->gyro_y = gyro_raw_y * dev->gyro_scale;
    data->gyro_z = gyro_raw_z * dev->gyro_scale;
    
    data->accel_x = accel_raw_x * dev->accel_scale;
    data->accel_y = accel_raw_y * dev->accel_scale;
    data->accel_z = accel_raw_z * dev->accel_scale;
    
    // Lire température (optionnel)
    uint8_t temp_raw[2];
    if (i2c_read_regs(dev, LSM6DSO32_REG_OUT_TEMP_L, temp_raw, 2)) {
        int16_t temp = (int16_t)((uint16_t)temp_raw[1] << 8 | temp_raw[0]);
        data->temperature = 25.0f + (temp / 256.0f);  // Formule datasheet
    } else {
        data->temperature = 0.0f;
    }
    
    return true;
}

// ============================================================================
// READ ACCEL ONLY
// ============================================================================
bool LSM6DSO32_read_accel(lsm6dso32_device_t* dev, float* x, float* y, float* z) {
    if (!dev || !dev->initialized) return false;
    
    uint8_t raw[6];
    if (!i2c_read_regs(dev, LSM6DSO32_REG_OUTX_L_A, raw, 6)) {
        return false;
    }
    
    int16_t ax = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
    int16_t ay = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
    int16_t az = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);
    
    if (x) *x = ax * dev->accel_scale;
    if (y) *y = ay * dev->accel_scale;
    if (z) *z = az * dev->accel_scale;
    
    return true;
}

// ============================================================================
// READ GYRO ONLY
// ============================================================================
bool LSM6DSO32_read_gyro(lsm6dso32_device_t* dev, float* x, float* y, float* z) {
    if (!dev || !dev->initialized) return false;
    
    uint8_t raw[6];
    if (!i2c_read_regs(dev, LSM6DSO32_REG_OUTX_L_G, raw, 6)) {
        return false;
    }
    
    int16_t gx = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
    int16_t gy = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
    int16_t gz = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);
    
    if (x) *x = gx * dev->gyro_scale;
    if (y) *y = gy * dev->gyro_scale;
    if (z) *z = gz * dev->gyro_scale;
    
    return true;
}

// ============================================================================
// RESET
// ============================================================================
bool LSM6DSO32_reset(lsm6dso32_device_t* dev) {
    if (!dev) return false;
    
    return i2c_write_reg(dev, LSM6DSO32_REG_CTRL3_C, 0x01);
}