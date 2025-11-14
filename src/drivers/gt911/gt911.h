#ifndef GT911_H
#define GT911_H

#include "src/touch/touch.h"
#include "constants.h"


typedef struct {
  uint8_t dev_addr;
} esp_lcd_touch_io_gt911_config_t;

typedef struct {
  uint16_t x[ESP_LCD_TOUCH_MAX_POINTS];
  uint16_t y[ESP_LCD_TOUCH_MAX_POINTS];
  uint8_t cnt;
} touch_gt911_point_t;

esp_err_t esp_lcd_touch_new_i2c_gt911(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch);

esp_lcd_touch_handle_t touch_gt911_init();

touch_gt911_point_t touch_gt911_read_point(uint8_t max_touch_cnt);

#define ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG() \
  { \
    .dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS, \
    .control_phase_bytes = 1, \
    .dc_bit_offset = 0, \
    .lcd_cmd_bits = 16, \
    .flags = { \
      .disable_control_phase = 1, \
    }, \
    .scl_speed_hz = 400 * 1000, \
  }

#endif