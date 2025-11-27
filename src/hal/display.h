/**
 * @file display.h
 * @brief Initialisation écran MIPI DSI + LVGL 9.3 + Touch GT911
 * 
 * Bibliothèques requises (dans ~/Arduino/libraries/) :
 * - ESP32_Display_Panel
 * - lvgl
 * - initGT911
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Wire.h>
#include <lvgl.h>
#include <esp_display_panel.hpp>
#include <initGT911.h>
#include "src/config/config.h"
#include "src/config/pins.h"
#include "src/system/lang.h"
#include "src/system/logger.h"


using namespace esp_panel::board;
using namespace esp_panel::drivers;

// =============================================================================
// VARIABLES GLOBALES
// =============================================================================
static Board* board = nullptr;
static lv_display_t* display = nullptr;
static lv_indev_t* touch_indev = nullptr;
static initGT911* touch = nullptr;

static lv_color_t* buf1 = nullptr;
static lv_color_t* buf2 = nullptr;

// =============================================================================
// CALLBACKS LVGL
// =============================================================================

/**
 * @brief Callback flush LVGL vers LCD
 */
static void display_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
  if (!board) {
    lv_display_flush_ready(disp);
    return;
  }

  auto lcd = board->getLCD();
  if (!lcd) {
    lv_display_flush_ready(disp);
    return;
  }

  int w = lv_area_get_width(area);
  int h = lv_area_get_height(area);
  lcd->drawBitmap(area->x1, area->y1, w, h, (const uint8_t*)px_map);

  lv_display_flush_ready(disp);
}

/**
 * @brief Callback touch LVGL
 */
static void display_touch_cb(lv_indev_t* indev, lv_indev_data_t* data) {
  if (!touch) {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }

  uint8_t count = touch->touched(GT911_MODE_POLLING);

  if (count > 0) {
    GTPoint p = touch->getPoint(0);
    data->point.x = p.x;
    data->point.y = p.y;
    LOG_V(LOG_TOUCH, "Touched");
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// =============================================================================
// FONCTIONS PUBLIQUES
// =============================================================================

/**
 * @brief Initialise le board (LCD + Backlight)
 */
bool display_init() {
  LOG_I(LOG_DISPLAY, "Initializing board...");

  board = new Board();
  if (!board) {
    LOG_E(LOG_DISPLAY, "Failed to create Board");
    return false;
  }

  if (!board->init()) {
    LOG_E(LOG_DISPLAY, "Board init failed");
    delete board;
    board = nullptr;
    return false;
  }

  if (!board->begin()) {
    LOG_E(LOG_DISPLAY, "Board begin failed");
    delete board;
    board = nullptr;
    return false;
  }

  auto backlight = board->getBacklight();
  if (backlight) {
    backlight->setBrightness(100);
  }

  LOG_I(LOG_DISPLAY, "Board OK");
  return true;
}

/**
 * @brief Initialise LVGL
 */
/**
 * @brief Initialise LVGL avec buffers FULL SCREEN
 */
bool display_init_lvgl() {
  LOG_I(LOG_DISPLAY, "Initializing LVGL (FULL MODE)...");

  lv_init();

  size_t buf_size = DISPLAY_WIDTH * DISPLAY_HEIGHT;
  size_t buf_bytes = buf_size * sizeof(lv_color_t);

  LOG_D(LOG_DISPLAY, "Allocating buffers: %d bytes each", buf_bytes);

  buf1 = (lv_color_t*)heap_caps_aligned_alloc(64, buf_bytes, MALLOC_CAP_SPIRAM);
  buf2 = (lv_color_t*)heap_caps_aligned_alloc(64, buf_bytes, MALLOC_CAP_SPIRAM);

  if (!buf1 || !buf2) {
    LOG_E(LOG_DISPLAY, "Buffer allocation failed");
    return false;
  }

  LOG_D(LOG_DISPLAY, "Buffer 1: 0x%08X", (uint32_t)buf1);
  LOG_D(LOG_DISPLAY, "Buffer 2: 0x%08X", (uint32_t)buf2);

  display = lv_display_create(DISPLAY_WIDTH, DISPLAY_HEIGHT);
  if (!display) {
    LOG_E(LOG_DISPLAY, "Display creation failed");
    return false;
  }

  lv_display_set_buffers(display, buf1, buf2, buf_bytes, LV_DISPLAY_RENDER_MODE_FULL);
  lv_display_set_flush_cb(display, display_flush_cb);

  lv_theme_default_init(display,
                        lv_palette_main(LV_PALETTE_BLUE),
                        lv_palette_main(LV_PALETTE_RED),
                        true, LV_FONT_DEFAULT);

  LOG_I(LOG_DISPLAY, "LVGL OK");
  return true;
}

/**
 * @brief Reset manuel du GT911 (pin INT non câblé)
 */
static void gt911_hardware_reset() {
  LOG_D(LOG_TOUCH, "Hardware reset via GPIO %d", TOUCH_RST);

  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, LOW);
  delay(20);  // Maintenir bas pendant 20ms
  digitalWrite(TOUCH_RST, HIGH);
  delay(100);  // Attendre que le GT911 boot
}

/**
 * @brief Scan I2C pour détecter le GT911
 */
static uint8_t gt911_scan_address() {
  const uint8_t addresses[] = { GT911_I2C_ADDR_BA, GT911_I2C_ADDR_28 };

  for (int i = 0; i < 2; i++) {
    Wire.beginTransmission(addresses[i]);
    if (Wire.endTransmission() == 0) {
      LOG_D(LOG_TOUCH, "GT911 detected at 0x%02X", addresses[i]);
      return addresses[i];
    }
  }

  return 0;  // Non trouvé
}

/**
 * @brief Initialise le tactile GT911 (sans pin INT)
 */
bool display_init_touch() {
  LOG_I(LOG_TOUCH, "Initializing GT911...");

  // S'assurer que Wire est initialisé
  Wire.begin();
  delay(50);

  // Reset hardware du GT911
  gt911_hardware_reset();

  // Scanner les adresses I2C pour trouver le GT911
  uint8_t detected_addr = gt911_scan_address();

  if (detected_addr == 0) {
    LOG_E(LOG_TOUCH, "GT911 not found at any address");
    return false;
  }

  LOG_I(LOG_TOUCH, "GT911 found at 0x%02X", detected_addr);

  // Créer instance avec l'adresse détectée
  touch = new initGT911(&Wire, detected_addr);
  if (!touch) {
    LOG_E(LOG_TOUCH, "Touch creation failed");
    return false;
  }

  // Initialiser SANS reset (pins à -1)
  // La bibliothèque ne fera pas de reset car les deux pins sont à -1
  if (!touch->begin(-1, -1, 400000)) {
    LOG_E(LOG_TOUCH, "Touch init failed");
    delete touch;
    touch = nullptr;
    return false;
  }

  // Configuration de l'écran
  touch->setupDisplay(DISPLAY_WIDTH, DISPLAY_HEIGHT, initGT911::Rotate::_0);

  // Créer input device LVGL
  touch_indev = lv_indev_create();
  lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(touch_indev, display_touch_cb);
  lv_indev_set_display(touch_indev, display);

  LOG_I(LOG_TOUCH, "Touch OK");
  return true;
}

/**
 * @brief Tâche LVGL (à appeler dans loop)
 */
void display_task() {
  static unsigned long last = 0;
  unsigned long now = millis();

  if (now - last >= 5) {
    lv_tick_inc(now - last);
    last = now;
    lv_timer_handler();
  }
}

/**
 * @brief Crée un écran de test simple
 */
void display_create_test_screen() {
  lv_obj_t* scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

  // Titre
  lv_obj_t* label_title = lv_label_create(scr);
  lv_label_set_text(label_title, trad[current_lang][0]);  // ← Simple !
  lv_obj_set_style_text_font(label_title, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(label_title, lv_color_hex(0x00FF00), 0);
  lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 40);

  // Message
  lv_obj_t* label_msg = lv_label_create(scr);
  lv_label_set_text(label_msg, trad[current_lang][1]);  // ← Simple !
  lv_obj_set_style_text_font(label_msg, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(label_msg, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(label_msg, LV_ALIGN_CENTER, 0, 0);

  // Bouton
  lv_obj_t* btn = lv_btn_create(scr);
  lv_obj_set_size(btn, 200, 80);
  lv_obj_align(btn, LV_ALIGN_CENTER, 0, 100);

  lv_obj_t* btn_label = lv_label_create(btn);
  lv_label_set_text(btn_label, trad[current_lang][2]);  // ← Simple !
  lv_obj_center(btn_label);

  lv_screen_load(scr);
}

bool init_display_gloabl() {
  bool disp = false;
  bool lvgl = false;
  bool touch = false;

  // Init Display Board
  disp = display_init();
  if (!disp) {
    LOG_E(LOG_DISPLAY, "FATAL: Display init failed");
    return false;
  }

  // Init LVGL
  lvgl = display_init_lvgl();
  if (!lvgl) {
    LOG_E(LOG_DISPLAY, "FATAL: LVGL init failed");
    return false;
  }

  // Attendre que le LCD soit prêt
  delay(100);

  touch = display_init_touch();
  // Init Touch (avec gestion automatique du reset et des deux adresses)
  if (!touch) {
    LOG_W(LOG_TOUCH, "Touch init failed");
    return false;
  }

  return true;
}

#endif  // DISPLAY_H