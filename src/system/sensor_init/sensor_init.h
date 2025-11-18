/**
 * @file sensor_init.h
 * @brief Initialisation centralisée de tous les capteurs
 * 
 * Gère l'initialisation du bus I2C et de tous les capteurs :
 * - LSM6DSO32 : IMU 6 axes (accéléromètre + gyroscope)
 * - BMP585 : Baromètre / altimètre
 * - GPS PA1010D : GPS I2C
 * 
 * @author Franck Moreau
 * @date 2025-11-15
 * @version 1.0
 * 
 * LOG DOCUMENTATION - MODULE: SYSTEM
 * [ERROR]
 *   - "I2C initialization failed" : Échec init bus I2C
 *   - "LSM6DSO32 not found at 0x%02X" : IMU non détecté
 *   - "BMP585 not found at 0x%02X" : Baromètre non détecté
 *   - "GPS PA1010D not found at 0x%02X" : GPS non détecté
 *   - "No sensors initialized!" : Aucun capteur n'a répondu
 * 
 * [WARNING]
 *   - "LSM6DSO32 initialization failed" : IMU trouvé mais config échouée
 *   - "BMP585 initialization failed" : Baro trouvé mais config échouée
 *   - "GPS initialization failed" : GPS trouvé mais config échouée
 *   - "Some sensors failed to initialize" : Au moins un capteur KO
 * 
 * [INFO]
 *   - "I2C initialized: SDA=%d SCL=%d freq=%d Hz" : Bus I2C OK
 *   - "I2C scan found %d device(s)" : Devices détectés sur bus
 *   - "LSM6DSO32 initialized: ±%dG, ±%d°/s" : IMU OK avec config
 *   - "BMP585 initialized: %dx temp, %dx press" : Baro OK avec config
 *   - "GPS PA1010D initialized" : GPS OK
 *   - "All sensors initialized successfully" : Tous capteurs OK
 * 
 * [VERBOSE]
 *   - "I2C device found at 0x%02X" : Device détecté lors du scan
 *   - "Configuring LSM6DSO32..." : Détails config IMU
 *   - "Configuring BMP585..." : Détails config baro
 */

#ifndef SENSOR_INIT_H
#define SENSOR_INIT_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DSO32.h>
#include "Adafruit_BMP5xx.h"
#include <Adafruit_GPS.h>

// Instances globales des capteurs
extern Adafruit_LSM6DSO32 lsm6dso32;
extern Adafruit_BMP5xx bmp585;
extern Adafruit_GPS gps;
extern TwoWire* gps_i2c;  // Pointeur vers Wire pour GPS I2C

// Status d'initialisation des capteurs
extern bool sensor_lsm6dso32_ready;
extern bool sensor_bmp585_ready;
extern bool sensor_gps_ready;

/**
 * @brief Initialise le bus I2C
 * 
 * Configure le bus I2C avec les pins et la fréquence définis
 * dans config.h et pins.h.
 * 
 * @return true si I2C initialisé, false si erreur
 */
bool sensor_init_i2c();

/**
 * @brief Scan le bus I2C et affiche les devices détectés
 * 
 * Utile pour le debug : vérifie que les capteurs sont bien
 * connectés et répond aux bonnes adresses.
 * 
 * @return Nombre de devices I2C détectés
 */
uint8_t sensor_scan_i2c();

/**
 * @brief Initialise le LSM6DSO32 (IMU)
 * 
 * Configure :
 * - Adresse I2C
 * - Plages accéléromètre et gyroscope
 * - Fréquences d'échantillonnage
 * - Filtres passe-bas
 * 
 * @return true si initialisé et configuré, false si erreur
 */
bool sensor_init_lsm6dso32();

/**
 * @brief Initialise le BMP585 (baromètre)
 * 
 * Configure :
 * - Adresse I2C
 * - Oversampling température et pression
 * - Filtre IIR
 * - Fréquence de sortie
 * 
 * @return true si initialisé et configuré, false si erreur
 */
bool sensor_init_bmp585();

/**
 * @brief Initialise le GPS PA1010D (I2C)
 * 
 * Configure :
 * - Adresse I2C
 * - Fréquence d'update (1 Hz)
 * - Sentences NMEA (RMC + GGA)
 * 
 * @return true si initialisé et configuré, false si erreur
 */
bool sensor_init_gps();

/**
 * @brief Initialise tous les capteurs I2C
 * 
 * Séquence complète :
 * 1. Initialisation bus I2C
 * 2. Scan bus I2C (debug)
 * 3. Initialisation LSM6DSO32
 * 4. Initialisation BMP585
 * 5. Initialisation GPS PA1010D
 * 6. Affichage résumé
 * 
 * @return true si au moins un capteur OK, false si tous KO
 */
bool sensor_init_all();

/**
 * @brief Affiche le résumé d'initialisation des capteurs
 * 
 * Log le status de chaque capteur et affiche un résumé
 * global de l'initialisation.
 */
void sensor_init_print_summary();

/**
 * @brief Vérifie si tous les capteurs critiques sont OK
 * 
 * Capteurs critiques : LSM6DSO32 + BMP585
 * Le GPS n'est pas critique (vario fonctionne sans).
 * 
 * @return true si LSM6DSO32 ET BMP585 OK, false sinon
 */
bool sensor_check_critical();

#endif // SENSOR_INIT_H