#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "Arduino.h"
#include "src/io_extension/io_extension.h"
#include "src/rgb_lcd_port/rgb_lcd_port.h"

#include "src/gt911/gt911.h"

static const char *TAG = "GT911";

#define ESP_LCD_TOUCH_GT911_READ_KEY_REG (0x8093)
#define ESP_LCD_TOUCH_GT911_READ_XY_REG (0x814E)
#define ESP_LCD_TOUCH_GT911_CONFIG_REG (0x8047)
#define ESP_LCD_TOUCH_GT911_PRODUCT_ID_REG (0x8140)
#define ESP_LCD_TOUCH_GT911_ENTER_SLEEP (0x8040)

#define ESP_GT911_TOUCH_MAX_BUTTONS (4)

esp_lcd_touch_handle_t tp_handle = NULL;

static esp_err_t esp_lcd_touch_gt911_read_data(esp_lcd_touch_handle_t tp);
static bool esp_lcd_touch_gt911_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
#if (ESP_LCD_TOUCH_MAX_BUTTONS > 0)
static esp_err_t esp_lcd_touch_gt911_get_button_state(esp_lcd_touch_handle_t tp, uint8_t n, uint8_t *state);
#endif
static esp_err_t esp_lcd_touch_gt911_del(esp_lcd_touch_handle_t tp);

static esp_err_t touch_gt911_i2c_read(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len);
static esp_err_t touch_gt911_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t data);


static esp_err_t touch_gt911_reset(esp_lcd_touch_handle_t tp);

static esp_err_t touch_gt911_read_cfg(esp_lcd_touch_handle_t tp);

static esp_err_t esp_lcd_touch_gt911_enter_sleep(esp_lcd_touch_handle_t tp);
static esp_err_t esp_lcd_touch_gt911_exit_sleep(esp_lcd_touch_handle_t tp);


esp_err_t esp_lcd_touch_new_i2c_gt911(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch) {
  esp_err_t ret = ESP_OK;

  if (!io || !config || !out_touch) {
#ifdef DEBUG_MODE
    ESP_LOGE(TAG, "Invalid arguments!");
#endif
    return ESP_ERR_INVALID_ARG;
  }

  esp_lcd_touch_handle_t esp_lcd_touch_gt911 =
    (esp_lcd_touch_handle_t)heap_caps_calloc(1, sizeof(esp_lcd_touch_t), MALLOC_CAP_DEFAULT);
  if (!esp_lcd_touch_gt911) {
#ifdef DEBUG_MODE
    ESP_LOGE(TAG, "Memory allocation failed for GT911 controller");
#endif
    return ESP_ERR_NO_MEM;
  }

  esp_lcd_touch_gt911->io = io;
  esp_lcd_touch_gt911->read_data = esp_lcd_touch_gt911_read_data;
  esp_lcd_touch_gt911->get_xy = esp_lcd_touch_gt911_get_xy;
#if (ESP_LCD_TOUCH_MAX_BUTTONS > 0)
  esp_lcd_touch_gt911->get_button_state = esp_lcd_touch_gt911_get_button_state;
#endif
  esp_lcd_touch_gt911->del = esp_lcd_touch_gt911_del;
  esp_lcd_touch_gt911->enter_sleep = esp_lcd_touch_gt911_enter_sleep;
  esp_lcd_touch_gt911->exit_sleep = esp_lcd_touch_gt911_exit_sleep;
  esp_lcd_touch_gt911->data.lock.owner = portMUX_FREE_VAL;
  memcpy(&esp_lcd_touch_gt911->config, config, sizeof(esp_lcd_touch_config_t));

  if (config->rst_gpio_num != GPIO_NUM_NC) {
    gpio_config_t rst_gpio_config = {
      .pin_bit_mask = BIT64(config->rst_gpio_num),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&rst_gpio_config);
    if (ret != ESP_OK) {
#ifdef DEBUG_MODE
      ESP_LOGE(TAG, "Failed to configure reset GPIO");
#endif
      heap_caps_free(esp_lcd_touch_gt911);
      return ret;
    }
  }

  if (config->int_gpio_num != GPIO_NUM_NC) {
    gpio_config_t int_gpio_config = {
      .pin_bit_mask = BIT64(config->int_gpio_num),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = config->levels.interrupt ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE,
    };
    ret = gpio_config(&int_gpio_config);
    if (ret != ESP_OK) {
#ifdef DEBUG_MODE
      ESP_LOGE(TAG, "Failed to configure interrupt GPIO");
#endif
      heap_caps_free(esp_lcd_touch_gt911);
      return ret;
    }
  }

  ret = touch_gt911_reset(esp_lcd_touch_gt911);
  if (ret != ESP_OK) {
#ifdef DEBUG_MODE
    ESP_LOGE(TAG, "GT911 reset failed");
#endif
    heap_caps_free(esp_lcd_touch_gt911);
    return ret;
  }

  ret = touch_gt911_read_cfg(esp_lcd_touch_gt911);
  if (ret != ESP_OK) {
#ifdef DEBUG_MODE
    ESP_LOGE(TAG, "GT911 configuration read failed");
#endif
    heap_caps_free(esp_lcd_touch_gt911);
    return ret;
  }

  *out_touch = esp_lcd_touch_gt911;
  return ESP_OK;
}

static esp_err_t esp_lcd_touch_gt911_enter_sleep(esp_lcd_touch_handle_t tp) {
  esp_err_t err = touch_gt911_i2c_write(tp, ESP_LCD_TOUCH_GT911_ENTER_SLEEP, 0x05);
  ESP_RETURN_ON_ERROR(err, TAG, "Enter Sleep failed!");

  return ESP_OK;
}

static esp_err_t esp_lcd_touch_gt911_exit_sleep(esp_lcd_touch_handle_t tp) {
  esp_err_t ret;
  esp_lcd_touch_handle_t esp_lcd_touch_gt911 = tp;

  if (esp_lcd_touch_gt911->config.int_gpio_num != GPIO_NUM_NC) {
    const gpio_config_t int_gpio_config_high = {
      .pin_bit_mask = BIT64(esp_lcd_touch_gt911->config.int_gpio_num),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&int_gpio_config_high);
    ESP_RETURN_ON_ERROR(ret, TAG, "High GPIO config failed");
    gpio_set_level(esp_lcd_touch_gt911->config.int_gpio_num, 1);

    vTaskDelay(pdMS_TO_TICKS(5));

    const gpio_config_t int_gpio_config_float = {
      .pin_bit_mask = BIT64(esp_lcd_touch_gt911->config.int_gpio_num),
      .mode = GPIO_MODE_OUTPUT_OD,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&int_gpio_config_float);
    ESP_RETURN_ON_ERROR(ret, TAG, "Float GPIO config failed");
  }

  return ESP_OK;
}

static esp_err_t esp_lcd_touch_gt911_read_data(esp_lcd_touch_handle_t tp) {
  esp_err_t err;
  uint8_t buf[41];
  uint8_t touch_cnt = 0;
  uint8_t clear = 0;
  size_t i = 0;

  assert(tp != NULL);

  err = touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, buf, 1);
  ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

  if ((buf[0] & 0x80) == 0x00) {
    touch_gt911_i2c_write(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, clear);
#if (ESP_LCD_TOUCH_MAX_BUTTONS > 0)
  } else if ((buf[0] & 0x10) == 0x10) {
    uint8_t key_max = ((ESP_GT911_TOUCH_MAX_BUTTONS < ESP_LCD_TOUCH_MAX_BUTTONS) ? (ESP_GT911_TOUCH_MAX_BUTTONS) : (ESP_LCD_TOUCH_MAX_BUTTONS));
    err = touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_READ_KEY_REG, &buf[0], key_max);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    touch_gt911_i2c_write(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, clear);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C write error!");

    portENTER_CRITICAL(&tp->data.lock);

    tp->data.buttons = key_max;
    for (i = 0; i < key_max; i++) {
      tp->data.button[i].status = buf[0] ? 1 : 0;
    }

    portEXIT_CRITICAL(&tp->data.lock);
#endif
  } else if ((buf[0] & 0x80) == 0x80) {
#if (ESP_LCD_TOUCH_MAX_BUTTONS > 0)
    portENTER_CRITICAL(&tp->data.lock);
    for (i = 0; i < ESP_LCD_TOUCH_MAX_BUTTONS; i++) {
      tp->data.button[i].status = 0;
    }
    portEXIT_CRITICAL(&tp->data.lock);
#endif
    touch_cnt = buf[0] & 0x0f;
    if (touch_cnt > 5 || touch_cnt == 0) {
      touch_gt911_i2c_write(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, clear);
      return ESP_OK;
    }

    err = touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG + 1, &buf[1], touch_cnt * 8);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    err = touch_gt911_i2c_write(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, clear);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    portENTER_CRITICAL(&tp->data.lock);

    touch_cnt = (touch_cnt > ESP_LCD_TOUCH_MAX_POINTS ? ESP_LCD_TOUCH_MAX_POINTS : touch_cnt);
    tp->data.points = touch_cnt;

    for (i = 0; i < touch_cnt; i++) {
      tp->data.coords[i].x = ((uint16_t)buf[(i * 8) + 3] << 8) + buf[(i * 8) + 2];
      tp->data.coords[i].y = (((uint16_t)buf[(i * 8) + 5] << 8) + buf[(i * 8) + 4]);
      tp->data.coords[i].strength = (((uint16_t)buf[(i * 8) + 7] << 8) + buf[(i * 8) + 6]);
    }

    portEXIT_CRITICAL(&tp->data.lock);
  }

  return ESP_OK;
}

static bool esp_lcd_touch_gt911_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num) {
  assert(tp != NULL);
  assert(x != NULL);
  assert(y != NULL);
  assert(point_num != NULL);
  assert(max_point_num > 0);

  portENTER_CRITICAL(&tp->data.lock);

  *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);

  for (size_t i = 0; i < *point_num; i++) {
    x[i] = tp->data.coords[i].x;
    y[i] = tp->data.coords[i].y;

    if (strength) {
      strength[i] = tp->data.coords[i].strength;
    }
  }

  tp->data.points = 0;

  portEXIT_CRITICAL(&tp->data.lock);

  return (*point_num > 0);
}

#if (ESP_LCD_TOUCH_MAX_BUTTONS > 0)
static esp_err_t esp_lcd_touch_gt911_get_button_state(esp_lcd_touch_handle_t tp, uint8_t n, uint8_t *state) {
  esp_err_t err = ESP_OK;
  assert(tp != NULL);
  assert(state != NULL);

  *state = 0;

  portENTER_CRITICAL(&tp->data.lock);

  if (n > tp->data.buttons) {
    err = ESP_ERR_INVALID_ARG;
  } else {
    *state = tp->data.button[n].status;
  }

  portEXIT_CRITICAL(&tp->data.lock);

  return err;
}
#endif

static esp_err_t esp_lcd_touch_gt911_del(esp_lcd_touch_handle_t tp) {
  assert(tp != NULL);

  if (tp->config.int_gpio_num != GPIO_NUM_NC) {
    gpio_reset_pin(tp->config.int_gpio_num);
    if (tp->config.interrupt_callback) {
      gpio_isr_handler_remove(tp->config.int_gpio_num);
    }
  }

  if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
    gpio_reset_pin(tp->config.rst_gpio_num);
  }

  free(tp);

  return ESP_OK;
}

esp_lcd_touch_handle_t touch_gt911_init() {
  esp_lcd_panel_io_handle_t tp_io_handle = NULL;
  const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();

  // Recuperer le bus I2C existant (deja initialise dans setup)
  DEV_I2C_Port port = DEV_I2C_Get_Handle();
  
  delay(10);

  pinMode(PIN_NUM_TOUCH_INT, OUTPUT);
  IO_EXTENSION_Output(IO_EXTENSION_IO_1, 0);

  delay(100);
  digitalWrite(PIN_NUM_TOUCH_INT, 0);

  delay(100);
  IO_EXTENSION_Output(IO_EXTENSION_IO_1, 1);

  delay(200);

#ifdef DEBUG_MODE
  ESP_LOGI(TAG, "Initialize I2C panel IO");
#endif

  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(port.bus, &tp_io_config, &tp_io_handle));

#ifdef DEBUG_MODE
  ESP_LOGI(TAG, "Initialize touch controller GT911");
#endif

  const esp_lcd_touch_config_t tp_cfg = {
    .x_max = LCD_H_RES,
    .y_max = LCD_V_RES,
    .rst_gpio_num = PIN_NUM_TOUCH_RST,
    .int_gpio_num = PIN_NUM_TOUCH_INT,
    .levels = {
      .reset = 0,
      .interrupt = 0,
    },
    .flags = {
      .swap_xy = 0,
      .mirror_x = 0,
      .mirror_y = 0,
    },
  };

  ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp_handle));

  return tp_handle;
}

touch_gt911_point_t touch_gt911_read_point(uint8_t max_touch_cnt) {
  touch_gt911_point_t data;

  esp_lcd_touch_read_data(tp_handle);
  esp_lcd_touch_get_coordinates(tp_handle, data.x, data.y, NULL, &data.cnt, max_touch_cnt);

  return data;
}

static esp_err_t touch_gt911_reset(esp_lcd_touch_handle_t tp) {
  assert(tp != NULL);

  if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
    ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, tp->config.levels.reset), TAG, "GPIO set level error!");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, !tp->config.levels.reset), TAG, "GPIO set level error!");
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return ESP_OK;
}

static esp_err_t touch_gt911_read_cfg(esp_lcd_touch_handle_t tp) {
  uint8_t buf[4];

  assert(tp != NULL);

  ESP_RETURN_ON_ERROR(touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_PRODUCT_ID_REG, (uint8_t *)&buf[0], 3), TAG, "GT911 read error!");
  ESP_RETURN_ON_ERROR(touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_CONFIG_REG, (uint8_t *)&buf[3], 1), TAG, "GT911 read error!");

#ifdef DEBUG_MODE
  ESP_LOGI(TAG, "TouchPad_ID:0x%02x,0x%02x,0x%02x", buf[0], buf[1], buf[2]);
  ESP_LOGI(TAG, "TouchPad_Config_Version:%d", buf[3]);
#endif

  return ESP_OK;
}

static esp_err_t touch_gt911_i2c_read(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len) {
  assert(tp != NULL);
  assert(data != NULL);

  return esp_lcd_panel_io_rx_param(tp->io, reg, data, len);
}

static esp_err_t touch_gt911_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t data) {
  assert(tp != NULL);

  uint8_t data_array[1] = { data };
  return esp_lcd_panel_io_tx_param(tp->io, reg, data_array, 1);
}