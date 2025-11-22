/**
 * @file LSM6DSO32_ESP32.h
 * @brief Driver LSM6DSO32 minimaliste avec lecture directe
 * 
 * IMU 6 axes (accéléromètre ±32G + gyroscope ±2000dps)
 * 
 * @author Franck Moreau
 * @date 2025-11-22
 * @version 1.0
 */

#ifndef LSM6DSO32_ESP32_H
#define LSM6DSO32_ESP32_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include "src/hal/i2c_wrapper/i2c_wrapper.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// REGISTRES LSM6DSO32
// ============================================================================
#define LSM6DSO32_REG_WHO_AM_I          0x0F
#define LSM6DSO32_REG_CTRL1_XL          0x10  // Accel config
#define LSM6DSO32_REG_CTRL2_G           0x11  // Gyro config
#define LSM6DSO32_REG_CTRL3_C           0x12  // Common config
#define LSM6DSO32_REG_CTRL6_C           0x15  // Accel high perf
#define LSM6DSO32_REG_CTRL7_G           0x16  // Gyro high perf
#define LSM6DSO32_REG_STATUS_REG        0x1E  // Status
#define LSM6DSO32_REG_OUT_TEMP_L        0x20  // Température
#define LSM6DSO32_REG_OUTX_L_G          0x22  // Gyro X (6 bytes)
#define LSM6DSO32_REG_OUTX_L_A          0x28  // Accel X (6 bytes)

// WHO_AM_I ID
#define LSM6DSO32_CHIP_ID               0x6C

// ============================================================================
// ACCEL RANGE (registre CTRL1_XL bits 2-3)
// ============================================================================
typedef enum {
    LSM6DSO32_ACCEL_RANGE_4G = 0,   // ±4G
    LSM6DSO32_ACCEL_RANGE_8G = 2,   // ±8G (recommandé parapente)
    LSM6DSO32_ACCEL_RANGE_16G = 3,  // ±16G
    LSM6DSO32_ACCEL_RANGE_32G = 1   // ±32G (max)
} lsm6dso32_accel_range_t;

// ============================================================================
// GYRO RANGE (registre CTRL2_G bits 1-3)
// ============================================================================
typedef enum {
    LSM6DSO32_GYRO_RANGE_125_DPS = 0x02,   // ±125 dps (max précision)
    LSM6DSO32_GYRO_RANGE_250_DPS = 0x00,   // ±250 dps
    LSM6DSO32_GYRO_RANGE_500_DPS = 0x04,   // ±500 dps
    LSM6DSO32_GYRO_RANGE_1000_DPS = 0x08,  // ±1000 dps
    LSM6DSO32_GYRO_RANGE_2000_DPS = 0x0C   // ±2000 dps
} lsm6dso32_gyro_range_t;

// ============================================================================
// DATA RATE (bits 4-7 de CTRL1_XL et CTRL2_G)
// ============================================================================
typedef enum {
    LSM6DSO32_RATE_OFF = 0x00,
    LSM6DSO32_RATE_12_5_HZ = 0x01,
    LSM6DSO32_RATE_26_HZ = 0x02,
    LSM6DSO32_RATE_52_HZ = 0x03,
    LSM6DSO32_RATE_104_HZ = 0x04,
    LSM6DSO32_RATE_208_HZ = 0x05,
    LSM6DSO32_RATE_416_HZ = 0x06,
    LSM6DSO32_RATE_833_HZ = 0x07,
    LSM6DSO32_RATE_1660_HZ = 0x08,
    LSM6DSO32_RATE_3330_HZ = 0x09,
    LSM6DSO32_RATE_6660_HZ = 0x0A
} lsm6dso32_data_rate_t;

// ============================================================================
// CONFIGURATION
// ============================================================================
typedef struct {
    i2c_bus_id_t bus;
    uint8_t address;
    
    lsm6dso32_accel_range_t accel_range;
    lsm6dso32_gyro_range_t gyro_range;
    lsm6dso32_data_rate_t accel_rate;
    lsm6dso32_data_rate_t gyro_rate;
} lsm6dso32_config_t;

// ============================================================================
// DONNÉES CAPTEUR
// ============================================================================
typedef struct {
    // Accéléromètre (m/s²)
    float accel_x;
    float accel_y;
    float accel_z;
    
    // Gyroscope (rad/s)
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    // Température (°C)
    float temperature;
} lsm6dso32_data_t;

// ============================================================================
// STRUCTURE DEVICE
// ============================================================================
typedef struct {
    i2c_bus_id_t bus;
    uint8_t address;
    bool initialized;
    
    // Facteurs de conversion
    float accel_scale;  // LSB → m/s²
    float gyro_scale;   // LSB → rad/s
} lsm6dso32_device_t;

// ============================================================================
// FONCTIONS PUBLIQUES
// ============================================================================

/**
 * @brief Initialise le LSM6DSO32
 * 
 * @param[out] dev Structure device
 * @param[in] config Configuration (range, rate)
 * @return true si succès
 */
bool LSM6DSO32_init(lsm6dso32_device_t* dev, const lsm6dso32_config_t* config);

/**
 * @brief Lit accéléromètre + gyroscope + température
 * 
 * @param[in] dev Device
 * @param[out] data Données lues
 * @return true si succès
 */
bool LSM6DSO32_read(lsm6dso32_device_t* dev, lsm6dso32_data_t* data);

/**
 * @brief Lit uniquement l'accéléromètre
 */
bool LSM6DSO32_read_accel(lsm6dso32_device_t* dev, float* x, float* y, float* z);

/**
 * @brief Lit uniquement le gyroscope
 */
bool LSM6DSO32_read_gyro(lsm6dso32_device_t* dev, float* x, float* y, float* z);

/**
 * @brief Soft reset du capteur
 */
bool LSM6DSO32_reset(lsm6dso32_device_t* dev);

#ifdef __cplusplus
}
#endif

#endif // LSM6DSO32_ESP32_H