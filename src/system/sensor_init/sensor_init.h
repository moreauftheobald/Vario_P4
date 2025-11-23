/**
 * @file sensor_init.h
 * @brief Initialisation centralisée - VERSION GPS PA1010D I2C + BMP5 + IMU
 * 
 * Initialise le GPS PA1010D, le BMP5 et l'IMU via i2c_wrapper
 * Le bus I2C doit être initialisé AVANT d'appeler ces fonctions
 *
 * Auteur : Franck Moreau
 */

#ifndef SENSOR_INIT_H
#define SENSOR_INIT_H

#include <Arduino.h>
#include "src/hal/i2c_wrapper/i2c_wrapper.h"
#include "src/system/GPS_I2C_ESP32/GPS_I2C_ESP32.h"
#include "src/system/BMP5XX_ESP32/BMP5XX_ESP32.h"
#include "src/system/MAX17048_ESP32/MAX17048_ESP32.h"
#include "config/config.h"

// Sélection IMU selon config.h
#if IMU_BNO08XX == 1
  #include "src/system/BNO08x_ESP32_P4/BNO08x_ESP32_P4.h"
#else
  #include "src/system/LSM6DSO32_ESP32/LSM6DSO32_ESP32.h"
#endif

// ================================
// === INSTANCES GLOBALES
// ================================

// GPS PA1010D I2C
extern gps_device_t gps_dev;
extern gps_data_t gps_data;
extern bool sensor_gps_ready;

// BMP5 Baromètre
extern bmp5_device_t bmp5_dev;
extern bool sensor_bmp5_ready;

// IMU (selon config)
#if IMU_BNO08XX == 1
  extern BNO08x_ESP32_P4* bno08x_dev;
#else
  extern lsm6dso32_device_t lsm6dso32_dev;
#endif
extern bool sensor_imu_ready;

// BMS MAX17048
extern max17048_device_t max17048_dev;
extern bool sensor_battery_ready;

// ================================
// === FONCTIONS PUBLIQUES
// ================================

/**
 * @brief Scan du bus I2C capteurs (port 1)
 * @return Nombre de périphériques détectés
 */
uint8_t sensor_scan_i2c();

/**
 * @brief Initialise le GPS PA1010D (I2C)
 * @return true si succès
 */
bool sensor_init_gps();

/**
 * @brief Initialise le BMP5 (I2C)
 * @return true si succès
 */
bool sensor_init_bmp5();

/**
 * @brief Initialise l'IMU (BNO08x ou LSM6DSO32 selon config)
 * @return true si succès
 */
bool sensor_init_imu();

/**
 * @brief Initialise le MAX17048 (I2C)
 * @return true si succès
 */
bool sensor_init_battery();

/**
 * @brief Lit et parse les données GPS
 * @return true si données parsées
 */
bool sensor_read_gps();

/**
 * @brief Lit les données BMP5
 * @return true si lecture réussie
 */
bool sensor_read_bmp5();

/**
 * @brief Lit BMP5 en mode rapide (pression seule)
 * @return true si lecture réussie
 */
bool sensor_read_bmp5_fast();

/**
 * @brief Lit les données IMU
 * @return true si lecture réussie
 */
bool sensor_read_imu();

/**
 * @brief Lit les données batterie
 * @return true si lecture réussie
 */
bool sensor_read_battery();

/**
 * @brief Affiche le statut GPS détaillé
 */
void sensor_test_gps();

/**
 * @brief Affiche le statut BMP5 détaillé
 */
void sensor_test_bmp5();

/**
 * @brief Affiche le statut IMU détaillé
 */
void sensor_test_imu();

/**
 * @brief Affiche le statut batterie détaillé
 */
void sensor_test_battery();

/**
 * @brief Initialise tous les capteurs
 * @return true si au moins un capteur OK
 */
bool sensor_init_all();

/**
 * @brief Affiche résumé état capteurs
 */
void sensor_init_print_summary();

#endif  // SENSOR_INIT_H