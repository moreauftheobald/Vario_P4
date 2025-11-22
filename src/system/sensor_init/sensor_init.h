/**
 * @file sensor_init.h
 * @brief Initialisation centralisée - VERSION GPS PA1010D I2C + BMP5
 * 
 * Initialise le GPS PA1010D et le BMP5 via i2c_wrapper
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
#include "src/system/LSM6DSO32_ESP32/LSM6DSO32_ESP32.h"
#include "src/system/MAX17048_ESP32/MAX17048_ESP32.h"

// ================================
// === INSTANCES GLOBALES
// ================================

// GPS PA1010D I2C (structure C)
extern gps_i2c_esp32_t gps;
extern bool sensor_gps_ready;

// BMP5 Baromètre (structure C)
extern bmp5_device_t bmp5_dev;  // ✅ Changé de bmp5_device à bmp5_dev
extern bool sensor_bmp5_ready;

// IMU LSM6DSO32 (structure simple)
extern lsm6dso32_device_t lsm6dso32_dev;
extern bool sensor_imu_ready;

//BMS MAX17048
extern max17048_device_t max17048_dev;
extern bool sensor_battery_ready;

// ================================
// === FONCTIONS PUBLIQUES
// ================================

/**
 * @brief Scan du bus I2C capteurs (port 1)
 *
 * Utilise i2c_wrapper pour scanner le bus I2C_BUS_1
 * Identifie automatiquement les périphériques connus
 * 
 * @return Nombre de périphériques détectés
 */
uint8_t sensor_scan_i2c();

/**
 * @brief Initialise le GPS PA1010D (I2C)
 *
 * - Utilise i2c_wrapper (bus déjà initialisé requis)
 * - Configure output: RMC + GGA
 * - Configure taux: 1Hz update + 1Hz fix
 * - Vérifie présence sur bus I2C
 * 
 * @return true si succès, false si erreur
 */
bool sensor_init_gps();

/**
 * @brief Initialise le BMP5 (I2C)
 *
 * - Utilise i2c_wrapper (bus déjà initialisé requis)
 * - Configure selon config.h (OSR, IIR, ODR)
 * - Lit coefficients de calibration NVM
 * - Démarre en mode continu
 * 
 * @return true si succès, false si erreur
 */
bool sensor_init_bmp5();

/**
 * @brief Lit et parse les données GPS
 * 
 * À appeler cycliquement (ex: dans loop ou tâche FreeRTOS)
 * Lit un caractère, parse si ligne complète reçue
 * 
 * @return true si données parsées, false sinon
 */
bool sensor_read_gps();

/**
 * @brief Lit les données BMP5
 * 
 * À appeler cycliquement (ex: dans loop ou tâche FreeRTOS)
 * Lit température et pression compensées
 * 
 * @return true si lecture réussie, false sinon
 */
bool sensor_read_bmp5();

/**
 * @brief Affiche le statut GPS détaillé
 * 
 * Affiche fix, satellites, position, altitude, time, etc.
 * Utile pour debugging
 */
void sensor_test_gps();

/**
 * @brief Affiche le statut BMP5 détaillé
 * 
 * Affiche température, pression, altitude calculée
 * Utile pour debugging
 */
void sensor_test_bmp5();

/**
 * @brief Initialise tous les capteurs
 * 
 * GPS + BMP5
 * Futur: LSM6DSO32
 * 
 * @return true si au moins un capteur OK, false si tous en échec
 */
bool sensor_init_all();

/**
 * @brief Affiche résumé état capteurs
 * 
 * Liste tous les capteurs avec leur statut (OK/FAIL)
 */
void sensor_init_print_summary();

void dump_all_bmp5_registers();

bool sensor_init_imu();
bool sensor_read_imu();
void sensor_test_imu();

// Fonctions
bool sensor_init_battery();
bool sensor_read_battery();
void sensor_test_battery();

#endif // SENSOR_INIT_H