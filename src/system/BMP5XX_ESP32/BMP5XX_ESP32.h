/**
 * @file BMP5XX_ESP32.h
 * @brief Driver BMP5XX (BMP581/BMP5) basé sur Adafruit
 * 
 * Adapté de la bibliothèque Adafruit_BMP5XX pour utiliser i2c_wrapper
 * 
 * @author Franck Moreau (adaptation)
 * @date 2025-11-21
 * @version 1.0
 */

#ifndef BMP5XX_ESP32_H
#define BMP5XX_ESP32_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include "src/hal/i2c_wrapper/i2c_wrapper.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// REGISTRES BMP5XX (depuis Adafruit_BMP5XX.h)
// ============================================================================
#define BMP5_REGISTER_CHIPID 0x01
#define BMP5_REGISTER_STATUS 0x28
#define BMP5_REGISTER_CALIB_DATA 0x20  // 21 bytes de calibration
#define BMP5_REGISTER_CMD 0x7E
#define BMP5_REGISTER_ODR_CONFIG 0x37
#define BMP5_REGISTER_OSR_CONFIG 0x36
#define BMP5_REGISTER_OSR_EFF 0x38
#define BMP5_REGISTER_INT_CONFIG 0x14
#define BMP5_REGISTER_DSP_CONFIG 0x30
#define BMP5_REGISTER_DSP_IIR 0x31

// Registres de données
#define BMP5_REGISTER_TEMP_DATA_XLSB 0x1D
#define BMP5_REGISTER_TEMP_DATA_LSB 0x1E
#define BMP5_REGISTER_TEMP_DATA_MSB 0x1F
#define BMP5_REGISTER_PRESS_DATA_XLSB 0x20
#define BMP5_REGISTER_PRESS_DATA_LSB 0x21
#define BMP5_REGISTER_PRESS_DATA_MSB 0x22

// Commandes
#define BMP5_CMD_SOFTRESET 0xB6
#define BMP5_CMD_NVM_READ 0xA2

// Chip IDs
#define BMP5_CHIPID_585 0x50
#define BMP5_CHIPID_581 0x51

// ============================================================================
// MODES ET CONFIGURATIONS
// ============================================================================
typedef enum {
    BMP5_MODE_SLEEP = 0x00,
    BMP5_MODE_FORCED = 0x01,
    BMP5_MODE_CONTINUOUS = 0x03
} bmp5_powermode_t;

typedef enum {
    BMP5_OVERSAMPLING_1X = 0x00,
    BMP5_OVERSAMPLING_2X = 0x01,
    BMP5_OVERSAMPLING_4X = 0x02,
    BMP5_OVERSAMPLING_8X = 0x03,
    BMP5_OVERSAMPLING_16X = 0x04,
    BMP5_OVERSAMPLING_32X = 0x05,
    BMP5_OVERSAMPLING_64X = 0x06,
    BMP5_OVERSAMPLING_128X = 0x07
} bmp5_oversampling_t;

typedef enum {
    BMP5_ODR_240_HZ = 0x00,
    BMP5_ODR_218_HZ = 0x01,
    BMP5_ODR_199_HZ = 0x02,
    BMP5_ODR_179_HZ = 0x03,
    BMP5_ODR_160_HZ = 0x04,
    BMP5_ODR_149_HZ = 0x05,
    BMP5_ODR_140_HZ = 0x06,
    BMP5_ODR_129_HZ = 0x07,
    BMP5_ODR_120_HZ = 0x08,
    BMP5_ODR_110_HZ = 0x09,
    BMP5_ODR_100_HZ = 0x0A,
    BMP5_ODR_89_HZ = 0x0B,
    BMP5_ODR_80_HZ = 0x0C,
    BMP5_ODR_70_HZ = 0x0D,
    BMP5_ODR_60_HZ = 0x0E,
    BMP5_ODR_50_HZ = 0x0F
} bmp5_odr_t;

typedef enum {
    BMP5_IIR_FILTER_OFF = 0x00,
    BMP5_IIR_FILTER_COEFF_1 = 0x01,
    BMP5_IIR_FILTER_COEFF_3 = 0x02,
    BMP5_IIR_FILTER_COEFF_7 = 0x03,
    BMP5_IIR_FILTER_COEFF_15 = 0x04,
    BMP5_IIR_FILTER_COEFF_31 = 0x05,
    BMP5_IIR_FILTER_COEFF_63 = 0x06,
    BMP5_IIR_FILTER_COEFF_127 = 0x07
} bmp5_iir_filter_t;

// ============================================================================
// STRUCTURES
// ============================================================================


/** Configuration du capteur */
typedef struct {
    i2c_bus_id_t bus;
    uint8_t address;
    bmp5_oversampling_t osr_temp;
    bmp5_oversampling_t osr_press;
    bmp5_odr_t odr;
    bmp5_iir_filter_t iir_filter;
    bmp5_powermode_t mode;
} bmp5_config_t;

/** Structure principale du capteur */
typedef struct {
    i2c_bus_id_t bus;
    uint8_t address;
    uint8_t chip_id;
    
    float temperature;
    float pressure;
    
    bmp5_oversampling_t osr_temp;
    bmp5_oversampling_t osr_press;
    bmp5_odr_t odr;
    bmp5_iir_filter_t iir_filter;
    bmp5_powermode_t mode;
    
    bool initialized;
    uint32_t last_read_ms;
} bmp5_t;

// ============================================================================
// FONCTIONS PUBLIQUES
// ============================================================================

/**
 * @brief Initialise le capteur BMP5XX
 * 
 * @param[out] bmp Structure BMP5
 * @param[in] config Configuration
 * @return true si succès, false si erreur
 */
bool BMP5_init(bmp5_t* bmp, const bmp5_config_t* config);

/**
 * @brief Soft reset du capteur
 * 
 * @param[in] bmp Structure BMP5
 * @return true si succès, false si erreur
 */
bool BMP5_reset(bmp5_t* bmp);

/**
 * @brief Lit température et pression
 * 
 * @param[in,out] bmp Structure BMP5 (mise à jour)
 * @return true si succès, false si erreur
 */
bool BMP5_read(bmp5_t* bmp);

/**
 * @brief Obtient la température compensée
 * 
 * @param[in] bmp Structure BMP5
 * @return Température en °C
 */
float BMP5_get_temperature(const bmp5_t* bmp);

/**
 * @brief Obtient la pression compensée
 * 
 * @param[in] bmp Structure BMP5
 * @return Pression en Pa
 */
float BMP5_get_pressure(const bmp5_t* bmp);

/**
 * @brief Obtient la pression en hPa
 * 
 * @param[in] bmp Structure BMP5
 * @return Pression en hPa
 */
float BMP5_get_pressure_hPa(const bmp5_t* bmp);

/**
 * @brief Calcule l'altitude
 * 
 * @param[in] bmp Structure BMP5
 * @param[in] qnh Pression au niveau de la mer (hPa)
 * @return Altitude en mètres
 */
float BMP5_calculate_altitude(const bmp5_t* bmp, float qnh);

/**
 * @brief Change le mode de fonctionnement
 * 
 * @param[in,out] bmp Structure BMP5
 * @param[in] mode Nouveau mode
 * @return true si succès, false si erreur
 */
bool BMP5_set_mode(bmp5_t* bmp, bmp5_powermode_t mode);

#ifdef __cplusplus
}
#endif

#endif // BMP5XX_ESP32_H