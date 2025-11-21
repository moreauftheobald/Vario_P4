/**
 * @file i2c_wrapper.h
 * @brief Wrapper I2C thread-safe pour ESP32-P4 - Support DUAL I2C
 * 
 * Utilise driver/i2c.h (même driver que ESP_Display_Panel) pour 
 * éviter les conflits avec le tactile GT911.
 * 
 * Gère 2 ports I2C indépendants:
 * - I2C_NUM_0 (port 0): Tactile GT911 (géré par ESP_Panel)
 * - I2C_NUM_1 (port 1): Capteurs (LSM6DSO32, BMP5, GPS)
 * 
 * @author Franck Moreau
 * @date 2025-11-19
 * @version 2.0
 * 
 * LOG DOCUMENTATION - MODULE: I2C
 * [ERROR]
 *   - "I2C%d init failed: %s" : Échec init bus I2C
 *   - "I2C%d write failed: %s" : Échec écriture
 *   - "I2C%d read failed: %s" : Échec lecture
 * 
 * [WARNING]
 *   - "I2C%d lock timeout" : Timeout acquisition mutex
 *   - "I2C%d device not responding" : Device pas de réponse
 * 
 * [INFO]
 *   - "I2C%d initialized: SCL=%d SDA=%d freq=%d" : Init réussie
 * 
 * [VERBOSE]
 *   - "I2C%d write addr=0x%02X len=%d" : Détails écriture
 *   - "I2C%d read addr=0x%02X len=%d" : Détails lecture
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


bool i2c_init(i2c_bus_id_t bus, const i2c_bus_config_t* config);
bool i2c_is_initialized(i2c_bus_id_t bus);
bool i2c_lock(i2c_bus_id_t bus, uint32_t timeout_ms);
void i2c_unlock(i2c_bus_id_t bus);
bool i2c_probe_device(i2c_bus_id_t bus, uint8_t device_addr);
bool i2c_write_bytes(i2c_bus_id_t bus, uint8_t device_addr, uint8_t reg_addr, const uint8_t* data, uint16_t length);
bool i2c_read_bytes(i2c_bus_id_t bus, uint8_t device_addr, uint8_t reg_addr, uint8_t* data, uint16_t length);
void i2c_deinit(i2c_bus_id_t bus);


#endif