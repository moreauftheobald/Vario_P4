/**
 * @file Vario_P4.ino
 * @brief Point d'entrée du variomètre ESP32-P4 avec GPS + USB MSC
 */

#include <Arduino.h>
#include "config/config.h"
#include "config/pins.h"
#include "src/hal/display_init.h"
#include "src/system/sd_manager/sd_manager.h"
#include "src/system/config_loader/config_loader.h"
#include "src/system/logger/logger.h"
#include "src/system/memory_monitor/memory_monitor.h"
#include "src/system/usb_msc_manager/usb_msc_manager.h"
#include "src/hal/i2c_wrapper/i2c_wrapper.h"
#include "src/system/sensor_init/sensor_init.h"
#include "src/data/config_data.h"

// Variable globale de configuration
variometer_config_t g_config = { 0 };

// Variables pour affichage GPS sur écran
lv_obj_t* label_gps_status = nullptr;
lv_obj_t* label_gps_fix = nullptr;
lv_obj_t* label_gps_position = nullptr;
lv_obj_t* label_usb_status = nullptr;
lv_obj_t* btn_usb_toggle = nullptr;

// Callback pour bouton USB toggle
void usb_toggle_callback(lv_event_t* e) {
  if (usb_msc_toggle()) {
    // Mettre à jour l'affichage
    if (usb_msc_is_active()) {
      lv_label_set_text(label_usb_status, "USB: ACTIVE - SD on PC");
      lv_obj_set_style_text_color(label_usb_status, lv_color_hex(0x00FF00), 0);
      lv_obj_set_style_bg_color(btn_usb_toggle, lv_palette_main(LV_PALETTE_RED), 0);
      lv_obj_t* btn_label = lv_obj_get_child(btn_usb_toggle, 0);
      lv_label_set_text(btn_label, "Stop USB");
    } else {
      lv_label_set_text(label_usb_status, "USB: IDLE");
      lv_obj_set_style_text_color(label_usb_status, lv_color_hex(0xFFFFFF), 0);
      lv_obj_set_style_bg_color(btn_usb_toggle, lv_palette_main(LV_PALETTE_GREEN), 0);
      lv_obj_t* btn_label = lv_obj_get_child(btn_usb_toggle, 0);
      lv_label_set_text(btn_label, "Start USB");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=================================");
  Serial.println("  Variometer ESP32-P4 Starting");
  Serial.println("=================================");

  // 1. Initialiser SD Manager
  Serial.println("[INIT] Initializing SD Manager...");
  if (!sd_manager_init()) {
    Serial.println("[INIT] SD Manager failed, continuing without SD");
  }

  // 2. Charger configuration
  Serial.println("[INIT] Loading configuration...");
  if (config_load()) {
    Serial.println("[INIT] Configuration loaded successfully");
    switch (g_config.config_source) {
      case CONFIG_SOURCE_SD:
        Serial.println("[INIT] Using configuration from SD card");
        break;
      case CONFIG_SOURCE_LITTLEFS:
        Serial.println("[INIT] Using configuration from LittleFS");
        break;
      case CONFIG_SOURCE_HARDCODED:
        Serial.println("[INIT] Using hardcoded configuration");
        break;
    }
  } else {
    Serial.println("[INIT] Configuration load failed, using defaults");
  }

  // 3. Initialiser logger
  Serial.println("[INIT] Initializing logger...");
  if (!logger_init()) {
    Serial.println("[INIT] Logger initialization failed");
  }

  // 4. Initialiser memory monitor
  Serial.println("[INIT] Initializing memory monitor...");
  memory_monitor_init();

  // 5. Initialiser I2C Bus 1 (capteurs)
  Serial.println("[INIT] Initializing I2C Bus 1 (sensors)...");
  i2c_bus_config_t cfg = {
    .sda_pin = I2C_SDA_PIN,
    .scl_pin = I2C_SCL_PIN,
    .frequency = I2C_FREQUENCY,
    .enabled = true
  };

  if (!i2c_init(I2C_BUS_1, &cfg)) {
    Serial.println("[FATAL] I2C Bus 1 init failed!");
    while (1) delay(1000);
  }
  Serial.println("[INIT] I2C Bus 1 initialized");

  // 6. Initialiser capteurs (GPS)
  Serial.println("[INIT] Initializing sensors...");
  if (!sensor_init_all()) {
    Serial.println("[WARNING] Sensor init had errors");
  }

  // 7. Initialiser USB MSC
  Serial.println("[INIT] Initializing USB MSC...");
  if (!usb_msc_init()) {
    Serial.println("[WARNING] USB MSC init failed");
  }

  // 8. Init Display
  if (!display_init_board()) {
    Serial.println("[FATAL] Board init failed!");
    while (1) delay(1000);
  }

  // 8. Init LVGL
  if (!display_init_lvgl()) {
    Serial.println("[FATAL] LVGL init failed!");
    while (1) delay(1000);
  }

  // 9. Créer UI avec infos GPS
  lv_obj_t* scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

  // Titre
  lv_obj_t* label_title = lv_label_create(scr);
  lv_label_set_text(label_title, "VARIOMETER READY");
  lv_obj_set_style_text_font(label_title, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(label_title, lv_color_hex(0x00FF00), 0);
  lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 20);

  // Statut GPS
  label_gps_status = lv_label_create(scr);
  lv_label_set_text(label_gps_status, "GPS: Initializing...");
  lv_obj_set_style_text_font(label_gps_status, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(label_gps_status, lv_color_hex(0xFFFF00), 0);
  lv_obj_align(label_gps_status, LV_ALIGN_CENTER, 0, -40);

  // Fix GPS
  label_gps_fix = lv_label_create(scr);
  lv_label_set_text(label_gps_fix, "Fix: NO | Sats: 0");
  lv_obj_set_style_text_font(label_gps_fix, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(label_gps_fix, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(label_gps_fix, LV_ALIGN_CENTER, 0, 0);

  // Position GPS
  label_gps_position = lv_label_create(scr);
  lv_label_set_text(label_gps_position, "Waiting for fix...");
  lv_obj_set_style_text_font(label_gps_position, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(label_gps_position, lv_color_hex(0xCCCCCC), 0);
  lv_obj_align(label_gps_position, LV_ALIGN_CENTER, 0, 40);

  // Statut USB
  label_usb_status = lv_label_create(scr);
  lv_label_set_text(label_usb_status, "USB: IDLE");
  lv_obj_set_style_text_font(label_usb_status, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(label_usb_status, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(label_usb_status, LV_ALIGN_BOTTOM_MID, 0, -80);

  // Bouton USB Toggle
  btn_usb_toggle = lv_btn_create(scr);
  lv_obj_set_size(btn_usb_toggle, 200, 50);
  lv_obj_align(btn_usb_toggle, LV_ALIGN_BOTTOM_MID, 0, -20);
  lv_obj_set_style_bg_color(btn_usb_toggle, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_obj_add_event_cb(btn_usb_toggle, usb_toggle_callback, LV_EVENT_CLICKED, NULL);

  lv_obj_t* btn_label = lv_label_create(btn_usb_toggle);
  lv_label_set_text(btn_label, "Start USB");
  lv_obj_center(btn_label);

  lv_screen_load(scr);
  lv_obj_update_layout(scr);
  lv_refr_now(g_display);

  Serial.println("===========================================");
  Serial.println("  READY - GPS LOOP + USB MSC");
  Serial.println("===========================================");
  Serial.println("Click 'Start USB' button to expose SD card via USB");
}

void loop() {
  static unsigned long last_gps_read = 0;
  static unsigned long last_gps_display = 0;
  static unsigned long last_heartbeat = 0;

  // 1. Tâche LVGL (critique, toutes les 5ms)
  display_task();

  unsigned long now = millis();

  // 2. Lire GPS continuellement (sauf si USB actif)
  if (now - last_gps_read >= 10) {  // Toutes les 10ms
    last_gps_read = now;
    
    if (sensor_gps_ready && !usb_msc_is_active()) {
      sensor_read_gps();
    }
  }

  // 3. Mettre à jour affichage GPS (toutes les 500ms)
  if (now - last_gps_display >= 500) {
    last_gps_display = now;
    
    if (sensor_gps_ready) {
      // Statut
      if (gps.fix) {
        lv_label_set_text(label_gps_status, "GPS: FIXED");
        lv_obj_set_style_text_color(label_gps_status, lv_color_hex(0x00FF00), 0);
      } else {
        lv_label_set_text(label_gps_status, "GPS: Searching...");
        lv_obj_set_style_text_color(label_gps_status, lv_color_hex(0xFFFF00), 0);
      }

      // Fix + satellites
      char buf_fix[64];
      snprintf(buf_fix, sizeof(buf_fix), 
               "Fix: %s | Sats: %d | HDOP: %.1f",
               gps.fix ? "YES" : "NO",
               gps.satellites,
               gps.HDOP);
      lv_label_set_text(label_gps_fix, buf_fix);

      // Position
      if (gps.fix) {
        char buf_pos[96];
        snprintf(buf_pos, sizeof(buf_pos),
                 "Lat: %.6f | Lon: %.6f\nAlt: %.1f m | Speed: %.1f kn",
                 gps.latitudeDegrees,
                 gps.longitudeDegrees,
                 gps.altitude,
                 gps.speed);
        lv_label_set_text(label_gps_position, buf_pos);
      } else {
        lv_label_set_text(label_gps_position, "Waiting for fix...");
      }
    } else {
      lv_label_set_text(label_gps_status, "GPS: ERROR");
      lv_obj_set_style_text_color(label_gps_status, lv_color_hex(0xFF0000), 0);
      lv_label_set_text(label_gps_fix, "GPS not initialized");
      lv_label_set_text(label_gps_position, "Check I2C connection");
    }
  }

  // 4. Heartbeat (toutes les 30s)
  if (now - last_heartbeat >= 30000) {
    last_heartbeat = now;
    
    // Skip si USB actif (SD pas accessible)
    if (usb_msc_is_active()) {
      LOG_I(LOG_MODULE_SYSTEM, "=== Heartbeat (USB active) ===");
      return;
    }
    
    LOG_I(LOG_MODULE_SYSTEM, "=== Heartbeat ===");
    memory_monitor_print_report();
    
    if (sd_manager_is_available()) {
      uint64_t free_space = sd_manager_free_space() / (1024 * 1024);
      uint64_t total_space = sd_manager_total_space() / (1024 * 1024);
      LOG_I(LOG_MODULE_STORAGE, "SD: %llu/%llu MB free", free_space, total_space);
    }

    // Test GPS détaillé
    if (sensor_gps_ready) {
      sensor_test_gps();
    }
  }

  delay(1);
}