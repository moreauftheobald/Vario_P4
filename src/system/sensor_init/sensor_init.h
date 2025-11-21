/**
 * @file sensor_init.h
 * @brief Initialisation centralisée - VERSION GPS PA1010D I2C
 * 
 * Initialise le GPS PA1010D via i2c_wrapper
 * Le bus I2C doit être initialisé AVANT d'appeler ces fonctions
 *
 * Auteur : Franck Moreau
 */

#ifndef SENSOR_INIT_H
#define SENSOR_INIT_H

#include <Arduino.h>
#include "src/hal/i2c_wrapper/i2c_wrapper.h"
#include "src/system/GPS_I2C_ESP32/GPS_I2C_ESP32.h"

// ================================
// === INSTANCES GLOBALES
// ================================

// GPS PA1010D I2C (structure C)
extern gps_i2c_esp32_t gps;
extern bool sensor_gps_ready;

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
 * @brief Lit et parse les données GPS
 * 
 * À appeler cycliquement (ex: dans loop ou tâche FreeRTOS)
 * Lit un caractère, parse si ligne complète reçue
 * 
 * @return true si données parsées, false sinon
 */
bool sensor_read_gps();

/**
 * @brief Affiche le statut GPS détaillé
 * 
 * Affiche fix, satellites, position, altitude, time, etc.
 * Utile pour debugging
 */
void sensor_test_gps();

/**
 * @brief Initialise tous les capteurs
 * 
 * Pour l'instant: GPS uniquement
 * Futur: LSM6DSO32, BMP585
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

#endif // SENSOR_INIT_H