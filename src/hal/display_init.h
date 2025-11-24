/**
 * @file display_init.h
 * @brief Initialisation écran MIPI DSI 7" + LVGL 9.3.0
 * 
 * @author Franck Moreau
 * @date 2025-11-19
 * @version 1.0
 */

#ifndef DISPLAY_INIT_H
#define DISPLAY_INIT_H

#include <Arduino.h>
#include "esp_display_panel.hpp"
#include <lvgl.h>

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// =============================================================================
// CONFIGURATION
// =============================================================================
#define DISPLAY_WIDTH 1024
#define DISPLAY_HEIGHT 600

// =============================================================================
// INSTANCES GLOBALES
// =============================================================================
extern Board* g_board;
extern lv_display_t* g_display;
extern lv_color_t* g_buf1;
extern lv_color_t* g_buf2;
extern bool g_touch_available;

// =============================================================================
// FONCTIONS PUBLIQUES
// =============================================================================

/**
 * @brief Initialise la carte (LCD + Touch + Backlight)
 * @return true si succès, false si erreur
 */
bool display_init_board();

/**
 * @brief Initialise LVGL avec buffers alignés
 * @return true si succès, false si erreur
 */
bool display_init_lvgl();

/**
 * @brief Callback flush LVGL
 */
void display_flush_callback(lv_display_t* disp, const lv_area_t* area, uint8_t* color_p);

/**
 * @brief Callback touch LVGL
 */
void display_touch_callback(lv_indev_t* indev, lv_indev_data_t* data);

/**
 * @brief Tâche LVGL (à appeler dans loop)
 */
void display_task();

// =============================================================================
// IMPLÉMENTATION
// =============================================================================

// Variables globales
Board* g_board = nullptr;
lv_display_t* g_display = nullptr;
lv_color_t* g_buf1 = nullptr;
lv_color_t* g_buf2 = nullptr;
bool g_touch_available = false;

// -----------------------------------------------------------------------------
// Callback Flush
// -----------------------------------------------------------------------------
void display_flush_callback(lv_display_t* disp, const lv_area_t* area, uint8_t* color_p) {
  if (!g_board) {
    lv_display_flush_ready(disp);
    return;
  }

  auto lcd = g_board->getLCD();
  if (lcd) {
    lcd->drawBitmap(area->x1, area->y1,
                    lv_area_get_width(area),
                    lv_area_get_height(area),
                    color_p);
  }

  lv_display_flush_ready(disp);
}

// -----------------------------------------------------------------------------
// Callback Touch
// -----------------------------------------------------------------------------
void display_touch_callback(lv_indev_t* indev, lv_indev_data_t* data) {

}

// -----------------------------------------------------------------------------
// Init Board
// -----------------------------------------------------------------------------
bool display_init_board() {
  Serial.println("[DISPLAY] Initializing board...");

  if (!psramFound()) {
    Serial.println("[DISPLAY] ERROR: PSRAM not available!");
    return false;
  }
  Serial.printf("[DISPLAY] PSRAM: %d bytes\n", ESP.getPsramSize());

  // Créer Board
  g_board = new Board();
  if (!g_board) {
    Serial.println("[DISPLAY] ERROR: Failed to create board");
    return false;
  }

  g_board->init();

  if (!g_board->begin()) {
    Serial.println("[DISPLAY] ERROR: Board begin failed");
    return false;
  }

  Serial.println("[DISPLAY] Board initialized");

  // Luminosité
  auto backlight = g_board->getBacklight();
  if (backlight) {
    backlight->setBrightness(200);
    Serial.println("[DISPLAY] Backlight set to 80%");
  }

  return true;
}

// -----------------------------------------------------------------------------
// Init LVGL
// -----------------------------------------------------------------------------
bool display_init_lvgl() {
  Serial.println("[LVGL] Initializing...");

  lv_init();

  auto lcd = g_board->getLCD();
  uint16_t width = lcd->getFrameWidth();
  uint16_t height = lcd->getFrameHeight();

  Serial.printf("[LVGL] LCD: %dx%d\n", width, height);

  // Buffers FULL en PSRAM alignés 64 bytes
  size_t buf_size = width * height;
  size_t buf_bytes = buf_size * sizeof(lv_color_t);

  Serial.printf("[LVGL] Allocating 2x %.2f MB buffers (64-byte aligned)...\n",
                buf_bytes / (1024.0f * 1024.0f));

  g_buf1 = (lv_color_t*)heap_caps_aligned_alloc(64, buf_bytes, MALLOC_CAP_SPIRAM);
  g_buf2 = (lv_color_t*)heap_caps_aligned_alloc(64, buf_bytes, MALLOC_CAP_SPIRAM);

  if (!g_buf1 || !g_buf2) {
    Serial.println("[LVGL] ERROR: Buffer allocation failed!");
    return false;
  }

  Serial.printf("[LVGL] Buffer 1: 0x%08X (aligned: %s)\n",
                (uint32_t)g_buf1, ((uint32_t)g_buf1 % 64 == 0) ? "YES" : "NO");
  Serial.printf("[LVGL] Buffer 2: 0x%08X (aligned: %s)\n",
                (uint32_t)g_buf2, ((uint32_t)g_buf2 % 64 == 0) ? "YES" : "NO");

  // Créer display
  g_display = lv_display_create(width, height);
  if (!g_display) {
    Serial.println("[LVGL] ERROR: Failed to create display");
    return false;
  }

  lv_display_set_buffers(g_display, g_buf1, g_buf2, buf_bytes, LV_DISPLAY_RENDER_MODE_FULL);
  lv_display_set_flush_cb(g_display, display_flush_callback);

  // Thème
  auto theme = lv_theme_default_init(g_display,
                                     lv_palette_main(LV_PALETTE_BLUE),
                                     lv_palette_main(LV_PALETTE_RED),
                                     true, LV_FONT_DEFAULT);
  lv_display_set_theme(g_display, theme);

  // Touch input
  lv_indev_t* indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, display_touch_callback);
  lv_indev_set_display(indev, g_display);

  Serial.println("[LVGL] Initialized successfully");

  return true;
}

// -----------------------------------------------------------------------------
// Task LVGL
// -----------------------------------------------------------------------------
void display_task() {
  static unsigned long last = 0;
  unsigned long now = millis();

  // Appeler lv_timer_handler() toutes les 5ms (critique !)
  if (now - last >= 5) {
    lv_tick_inc(now - last);  // ← Utiliser le delta réel
    last = now;

    lv_timer_handler();  // ← LVGL traite les événements ici
  }
}

#endif  // DISPLAY_INIT_H