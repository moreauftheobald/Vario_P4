/**
 * @file BMP5XX_ESP32.h
 * @brief Driver BMP581 minimaliste avec lecture/écriture directe
 * 
 * @author Franck Moreau
 * @date 2025-11-22
 * @version 3.0 (simplifié)
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
// REGISTRES BMP581
// ============================================================================
#define BMP5_REG_CHIP_ID 0x01
#define BMP5_REG_STATUS 0x28
#define BMP5_REG_DSP_CONFIG 0x30
#define BMP5_REG_DSP_IIR 0x31
#define BMP5_REG_OSR_CONFIG 0x36
#define BMP5_REG_ODR_CONFIG 0x37
#define BMP5_REG_TEMP_XLSB 0x1D
#define BMP5_REG_PRESS_XLSB 0x20
#define BMP5_REG_CMD 0x7E

// Commandes
#define BMP5_CMD_SOFT_RESET 0xB6

// Chip ID
#define BMP5_CHIPID_581 0x51

  // ============================================================================
  // OVERSAMPLING (valeurs pour OSR_CONFIG)
  // ============================================================================
  typedef enum {
    BMP5_OSR_1X = 0,
    BMP5_OSR_2X = 1,
    BMP5_OSR_4X = 2,
    BMP5_OSR_8X = 3,
    BMP5_OSR_16X = 4,
    BMP5_OSR_32X = 5,
    BMP5_OSR_64X = 6,
    BMP5_OSR_128X = 7
  } bmp5_osr_t;

  // ============================================================================
  // IIR FILTER (valeurs pour DSP_IIR)
  // ============================================================================
  typedef enum {
    BMP5_IIR_BYPASS = 0,
    BMP5_IIR_COEFF_1 = 1,
    BMP5_IIR_COEFF_3 = 2,
    BMP5_IIR_COEFF_7 = 3,
    BMP5_IIR_COEFF_15 = 4,
    BMP5_IIR_COEFF_31 = 5,
    BMP5_IIR_COEFF_63 = 6,
    BMP5_IIR_COEFF_127 = 7
  } bmp5_iir_t;

  // ============================================================================
  // OUTPUT DATA RATE (valeurs pour ODR_CONFIG)
  // ============================================================================
  typedef enum {
    BMP5_ODR_240_HZ = 0x00,
    BMP5_ODR_160_HZ = 0x04,
    BMP5_ODR_100_HZ = 0x0A,
    BMP5_ODR_50_HZ = 0x0F,
    BMP5_ODR_25_HZ = 0x14,
    BMP5_ODR_10_HZ = 0x17,
    BMP5_ODR_5_HZ = 0x18,
    BMP5_ODR_1_HZ = 0x1C
  } bmp5_odr_t;

  // ============================================================================
  // CONFIGURATION
  // ============================================================================
  typedef struct {
    i2c_bus_id_t bus;
    uint8_t address;

    bmp5_osr_t osr_temp;
    bmp5_osr_t osr_press;
    bmp5_iir_t iir_filter;
    bmp5_odr_t odr;
  } bmp5_config_t;

  // ============================================================================
  // DONNÉES CAPTEUR
  // ============================================================================
  typedef struct {
    float temperature;  // °C
    float pressure;     // hPa
    float altitude;     // m (calculée avec QNH)
  } bmp5_data_t;

  // ============================================================================
  // STRUCTURE DEVICE
  // ============================================================================
  typedef struct {
    i2c_bus_id_t bus;
    uint8_t address;
    bool initialized;
  } bmp5_device_t;

  // ============================================================================
  // FONCTIONS PUBLIQUES
  // ============================================================================

  /**
 * @brief Initialise le BMP581 avec configuration
 * 
 * @param[out] dev Structure device
 * @param[in] config Configuration (OSR, IIR, ODR)
 * @return true si succès
 */
  bool BMP5_init(bmp5_device_t* dev, const bmp5_config_t* config);

  /**
 * @brief Lit température + pression + calcule altitude
 * 
 * @param[in] dev Device
 * @param[out] data Données lues
 * @param[in] qnh QNH en hPa pour calcul altitude (ex: 1013.25)
 * @return true si succès
 */
  bool BMP5_read(bmp5_device_t* dev, bmp5_data_t* data, float qnh);

  /**
 * @brief Soft reset du capteur
 */
  bool BMP5_reset(bmp5_device_t* dev);

  /**
 * @brief Lit uniquement la pression (3 bytes au lieu de 6)
 * 
 * Optimisation pour variomètre : on lit souvent la pression
 * sans avoir besoin de la température.
 * 
 * @param[in] dev Device
 * @param[out] pressure Pression en hPa
 * @return true si succès
 */
  bool BMP5_read_pressure_only(bmp5_device_t* dev, float* pressure);

  /**
 * @brief Lit uniquement la température (3 bytes au lieu de 6)
 * 
 * @param[in] dev Device
 * @param[out] temperature Température en °C
 * @return true si succès
 */
  bool BMP5_read_temperature_only(bmp5_device_t* dev, float* temperature);


#ifdef __cplusplus
}
#endif

#endif  // BMP5XX_ESP32_H