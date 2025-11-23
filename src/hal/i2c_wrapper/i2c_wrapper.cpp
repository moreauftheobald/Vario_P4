/**
 * @file i2c_wrapper.cpp
 * @brief Wrapper I2C modifié pour compatibilité BNO080
 */

#include "i2c_wrapper.h"
#include <driver/i2c.h>

typedef struct {
  SemaphoreHandle_t mutex;
  bool initialized;
  i2c_bus_config_t config;
} i2c_port_state_t;

static i2c_port_state_t i2c_ports[I2C_BUS_MAX];

static i2c_port_t to_driver_port(i2c_bus_id_t bus) {
  return (bus == I2C_BUS_0) ? I2C_NUM_0 : I2C_NUM_1;
}

bool i2c_init(i2c_bus_id_t bus, const i2c_bus_config_t *config) {
  i2c_config_t cfg = {};
  cfg.mode = I2C_MODE_MASTER;
  cfg.sda_io_num = (gpio_num_t)config->sda_pin;
  cfg.scl_io_num = (gpio_num_t)config->scl_pin;
  cfg.master.clk_speed = config->frequency;
  cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  
  // Configuration spéciale pour BNO080
  cfg.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;  // Source clock normale

  if (i2c_param_config(to_driver_port(bus), &cfg) != ESP_OK)
    return false;

  if (i2c_driver_install(to_driver_port(bus), cfg.mode, 0, 0, 0) != ESP_OK)
    return false;
  
  // Timeout adapté pour BNO080 (utilise beaucoup de clock stretching)
  // Valeur en cycles APB (80MHz) : ~200ms
  i2c_set_timeout(to_driver_port(bus), 16000000);  

  i2c_ports[bus].mutex = xSemaphoreCreateMutex();
  i2c_ports[bus].initialized = true;
  i2c_ports[bus].config = *config;

  return true;
}

bool i2c_is_initialized(i2c_bus_id_t bus) {
  return (bus < I2C_BUS_MAX && i2c_ports[bus].initialized);
}

bool i2c_lock(i2c_bus_id_t bus, uint32_t timeout_ms) {
  if (!i2c_is_initialized(bus)) return false;
  TickType_t t = timeout_ms ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY;
  return xSemaphoreTake(i2c_ports[bus].mutex, t) == pdTRUE;
}

void i2c_unlock(i2c_bus_id_t bus) {
  if (i2c_is_initialized(bus)) {
    xSemaphoreGive(i2c_ports[bus].mutex);
  }
}

bool i2c_probe_device(i2c_bus_id_t bus, uint8_t addr) {
  if (!i2c_lock(bus, I2C_TIMEOUT_MS)) return false;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(to_driver_port(bus), cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  i2c_unlock(bus);

  return ret == ESP_OK;
}

// Fonction spéciale pour le BNO080 avec gestion du NACK initial
bool i2c_probe_bno080(i2c_bus_id_t bus, uint8_t addr) {
  if (!i2c_lock(bus, I2C_TIMEOUT_MS)) return false;

  // Le BNO080 peut NACKer au début, on essaie plusieurs fois
  bool found = false;
  for (int retry = 0; retry < 5; retry++) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // Important : ne pas vérifier l'ACK pour le BNO080
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, false);  // false = pas de check ACK
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(to_driver_port(bus), cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK || ret == ESP_ERR_TIMEOUT) {
      found = true;
      break;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  i2c_unlock(bus);
  return found;
}