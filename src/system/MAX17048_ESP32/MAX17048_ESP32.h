/**
 * @file MAX17048_ESP32.h
 * @brief Driver MAX17048 minimaliste - Jauge batterie LiPo
 * 
 * Mesure voltage et % charge batterie via I2C
 * 
 * @author Franck Moreau
 * @date 2025-11-22
 * @version 1.0
 */

#ifndef MAX17048_ESP32_H
#define MAX17048_ESP32_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include "src/hal/i2c_wrapper/i2c_wrapper.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// REGISTRES MAX17048
// ============================================================================
#define MAX17048_REG_VCELL      0x02  // Voltage cellule (16-bit)
#define MAX17048_REG_SOC        0x04  // State of Charge (16-bit)
#define MAX17048_REG_MODE       0x06  // Mode config
#define MAX17048_REG_VERSION    0x08  // Chip version
#define MAX17048_REG_CONFIG     0x0C  // Configuration
#define MAX17048_REG_COMMAND    0xFE  // Commande spéciale

// Adresse I2C (fixe)
#define MAX17048_I2C_ADDR       0x36

// Commandes
#define MAX17048_CMD_RESET      0x5400  // Soft reset

// ============================================================================
// CONFIGURATION
// ============================================================================
typedef struct {
    i2c_bus_id_t bus;
    uint8_t address;
    
    uint8_t alert_threshold;  // Seuil alerte batterie faible (%)
} max17048_config_t;

// ============================================================================
// DONNÉES BATTERIE
// ============================================================================
typedef struct {
    float voltage;      // Voltage batterie (V)
    float soc;          // State of Charge (%)
    float charge_rate;  // Taux de charge/décharge (%/h)
    bool is_charging;   // En charge
    bool alert;         // Alerte batterie faible
} max17048_data_t;

// ============================================================================
// STRUCTURE DEVICE
// ============================================================================
typedef struct {
    i2c_bus_id_t bus;
    uint8_t address;
    bool initialized;
    uint16_t version;
} max17048_device_t;

// ============================================================================
// FONCTIONS PUBLIQUES
// ============================================================================

/**
 * @brief Initialise le MAX17048
 * 
 * @param[out] dev Structure device
 * @param[in] config Configuration
 * @return true si succès
 */
bool MAX17048_init(max17048_device_t* dev, const max17048_config_t* config);

/**
 * @brief Lit voltage + SOC + status
 * 
 * @param[in] dev Device
 * @param[out] data Données lues
 * @return true si succès
 */
bool MAX17048_read(max17048_device_t* dev, max17048_data_t* data);

/**
 * @brief Lit uniquement le voltage
 * 
 * @param[in] dev Device
 * @return Voltage en V (0.0 si erreur)
 */
float MAX17048_read_voltage(max17048_device_t* dev);

/**
 * @brief Lit uniquement le SOC
 * 
 * @param[in] dev Device
 * @return SOC en % (0.0 si erreur)
 */
float MAX17048_read_soc(max17048_device_t* dev);

/**
 * @brief Configure le seuil d'alerte batterie faible
 * 
 * @param[in] dev Device
 * @param[in] threshold Seuil en % (1-32)
 * @return true si succès
 */
bool MAX17048_set_alert_threshold(max17048_device_t* dev, uint8_t threshold);

/**
 * @brief Reset du capteur
 * 
 * @param[in] dev Device
 * @return true si succès
 */
bool MAX17048_reset(max17048_device_t* dev);

/**
 * @brief Quick start pour recalibration rapide
 * 
 * Utile après changement de batterie
 * 
 * @param[in] dev Device
 * @return true si succès
 */
bool MAX17048_quick_start(max17048_device_t* dev);

#ifdef __cplusplus
}
#endif

#endif // MAX17048_ESP32_H