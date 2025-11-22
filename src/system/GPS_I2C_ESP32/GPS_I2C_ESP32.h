/**
 * @file GPS_I2C_ESP32.h
 * @brief Driver GPS PA1010D minimaliste - I2C
 * 
 * Parse uniquement RMC et GGA (essentiel pour variomètre)
 * 
 * @author Franck Moreau
 * @date 2025-11-22
 * @version 2.0 (simplifié)
 */

#ifndef GPS_I2C_ESP32_H
#define GPS_I2C_ESP32_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include "src/hal/i2c_wrapper/i2c_wrapper.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// CONSTANTES
// ============================================================================
#define GPS_I2C_ADDR 0x10
#define GPS_I2C_MAX_TRANSFER 32
#define GPS_MAX_LINE_LEN 120
// Registres de longueur de données disponibles
#define GPS_REG_DATA_LEN_MSB 0xFD
#define GPS_REG_DATA_LEN_LSB 0xFE

// Commandes PMTK simplifiées
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

  // ============================================================================
  // STRUCTURES
  // ============================================================================

  /**
 * @brief Configuration GPS
 */
  typedef struct {
    i2c_bus_id_t bus;
    uint8_t address;
  } gps_i2c_config_t;

  /**
 * @brief Données GPS (uniquement l'essentiel)
 */
  typedef struct {
    // Position
    float latitude;   // Degrés décimaux (+ = Nord)
    float longitude;  // Degrés décimaux (+ = Est)
    float altitude;   // Altitude GPS (m)

    // Vitesse & cap
    float speed;   // Vitesse sol (knots)
    float course;  // Cap (degrés)

    // Qualité
    bool fix;            // Fix valide
    uint8_t satellites;  // Nombre de satellites
    float hdop;          // Dilution précision horizontale

    // Heure UTC
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;

    // Date UTC
    uint8_t day;
    uint8_t month;
    uint8_t year;  // 2 chiffres (ex: 25 pour 2025)

    // Timestamp dernière mise à jour
    uint32_t last_update_ms;
  } gps_data_t;

  /**
 * @brief Structure device GPS
 */
  typedef struct {
    i2c_bus_id_t bus;
    uint8_t address;
    bool initialized;

    // Buffer ligne NMEA en cours de lecture
    char line_buffer[GPS_MAX_LINE_LEN];
    uint8_t line_index;

    // Dernier caractère lu (pour filtrage newlines)
    uint8_t last_char;
  } gps_device_t;

  // ============================================================================
  // FONCTIONS PUBLIQUES
  // ============================================================================

  /**
 * @brief Initialise le GPS PA1010D
 * 
 * Configure:
 * - Output RMC + GGA (uniquement)
 * - Update rate 1Hz
 * 
 * @param[out] dev Structure device
 * @param[in] config Configuration
 * @return true si succès
 */
  bool GPS_init(gps_device_t* dev, const gps_i2c_config_t* config);

  /**
 * @brief Lit et parse les données GPS
 * 
 * À appeler en polling (ex: toutes les 10ms).
 * Lit les données I2C, construit les lignes NMEA et les parse.
 * 
 * @param[in] dev Device GPS
 * @param[out] data Données GPS mises à jour si ligne complète parsée
 * @return true si ligne NMEA parsée avec succès, false sinon
 */
  bool GPS_read(gps_device_t* dev, gps_data_t* data);

  /**
 * @brief Envoie une commande PMTK au GPS
 * 
 * @param[in] dev Device GPS
 * @param[in] cmd Commande PMTK (avec checksum)
 * @return true si envoi réussi
 */
  bool GPS_send_command(gps_device_t* dev, const char* cmd);

  /**
 * @brief Vérifie si le GPS a un fix valide
 * 
 * @param[in] data Données GPS
 * @return true si fix valide
 */
  static inline bool GPS_has_fix(const gps_data_t* data) {
    return data->fix && data->satellites > 0;
  }

  /**
 * @brief Calcule le temps écoulé depuis dernière mise à jour
 * 
 * @param[in] data Données GPS
 * @return Temps écoulé en secondes
 */
  static inline float GPS_time_since_update(const gps_data_t* data) {
    return (millis() - data->last_update_ms) / 1000.0f;
  }

#ifdef __cplusplus
}
#endif

#endif  // GPS_I2C_ESP32_H