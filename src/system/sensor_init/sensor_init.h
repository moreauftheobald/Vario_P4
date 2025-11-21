/**
 * @file sensor_init.h
 * @brief Initialisation centralisée - VERSION GPS UNIQUEMENT
 * 
 * Tout ce qui ne concerne pas le GPS a été volontairement commenté
 * afin de conserver la structure originale sans casser ton projet.
 *
 * Le GPS utilise désormais :
 *  - i2c_wrapper
 *  - ta bibliothèque GPS_I2C_ESP32 mise à jour
 *  - bus I2C déjà initialisé en amont
 * 
 * Auteur : Franck Moreau
 */

#ifndef SENSOR_INIT_H
#define SENSOR_INIT_H

#include <Arduino.h>

// ================================
// === COMMENTÉ : AUTRES CAPTEURS
// ================================
/*
#include <Wire.h>
#include <Adafruit_LSM6DSO32.h>
#include "Adafruit_BMP5xx.h"
*/

// ================================
// === GPS I2C + I2C_WRAPPER
// ================================
#include "src/hal/i2c_wrapper/i2c_wrapper.h"
#include "src/system/GPS_I2C_ESP32/GPS_I2C_ESP32.h"


// ================================
// === INSTANCES GLOBALES
// ================================

// COMMENTÉ : capteurs non GPS
/*
extern Adafruit_LSM6DSO32 lsm6dso32;
extern Adafruit_BMP5xx bmp585;
*/

// GPS I2C PA1010D
extern GPS_I2C_ESP32 gps;

// Status d'initialisation
/*
extern bool sensor_lsm6dso32_ready;
extern bool sensor_bmp585_ready;
*/

extern bool sensor_gps_ready;


// ================================
// === FONCTIONS CONSERVÉES / GPS
// ================================

/**
 * @brief Scan du bus I2C via i2c_wrapper
 *
 * Utilise le port I2C_BUS_1 (capteurs)
 */
uint8_t sensor_scan_i2c();


/**
 * @brief Initialise le GPS PA1010D (I2C)
 *
 * - Utilise i2c_wrapper
 * - Le bus est déjà initialisé en amont
 * - Address par défaut : 0x10
 */
bool sensor_init_gps();


// ================================
// === COMMENTÉ : AUTRES FONCTIONS
// ================================

/*
bool sensor_init_i2c();
bool sensor_init_lsm6dso32();
bool sensor_init_bmp585();
*/


/**
 * @brief Initialise uniquement le GPS
 */
bool sensor_init_all();


/**
 * @brief Affiche l'état du GPS
 */
void sensor_init_print_summary();


// ================================
// === COMMENTÉ : CRITICAL CHECK
// ================================
/*
bool sensor_check_critical();
*/

#endif // SENSOR_INIT_H
