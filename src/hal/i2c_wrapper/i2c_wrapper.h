/**
 * @file i2c_wrapper.h
 * @brief Wrapper I2C thread-safe pour ESP32-P4 - Support DUAL I2C
 */

#ifndef I2C_WRAPPER_H
#define I2C_WRAPPER_H

#include <Arduino.h>
#include "driver/i2c.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define I2C_TIMEOUT_MS 1000

typedef enum {
  I2C_BUS_0 = 0,
  I2C_BUS_1 = 1,
  I2C_BUS_MAX
} i2c_bus_id_t;

typedef struct {
  uint8_t sda_pin;
  uint8_t scl_pin;
  uint32_t frequency;
  bool enabled;
} i2c_bus_config_t;

/**
 * @brief Initialise un bus I2C
 */
bool i2c_init(i2c_bus_id_t bus, const i2c_bus_config_t* config);

/**
 * @brief Vérifie si un bus I2C est initialisé
 */
bool i2c_is_initialized(i2c_bus_id_t bus);

/**
 * @brief Acquiert le verrou du bus I2C (thread-safe)
 * 
 * DOIT être appelé avant toute transaction I2C.
 * Toujours appeler i2c_unlock() après usage.
 * 
 * @param[in] bus Bus I2C
 * @param[in] timeout_ms Timeout en ms (0 = infini)
 * @return true si verrou acquis, false si timeout
 */
bool i2c_lock(i2c_bus_id_t bus, uint32_t timeout_ms);

/**
 * @brief Libère le verrou du bus I2C
 * 
 * DOIT être appelé après chaque i2c_lock() réussi.
 * 
 * @param[in] bus Bus I2C
 */
void i2c_unlock(i2c_bus_id_t bus);

/**
 * @brief Teste la présence d'un périphérique I2C
 * 
 * Gère automatiquement le lock/unlock.
 */
bool i2c_probe_device(i2c_bus_id_t bus, uint8_t device_addr);

#endif