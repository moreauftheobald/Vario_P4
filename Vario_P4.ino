/**
 * @file Vario_P4.ino
 * @brief Point d'entrée du variomètre ESP32-P4 avec task_flight
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
#include "src/system/sensor_init/sensor_init.h"
#include "src/data/config_data.h"
#include "src/tasks/task_flight.h"

// Variables globales
variometer_config_t g_config = { 0 };
flight_data_t g_flight_data = { 0 };
lv_obj_t* label_flight = nullptr;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("===========================================");
  Serial.println("  VARIOMETER ESP32-P4 - BOOT");
  Serial.println("===========================================");

  // 1. Reset GT911 AVANT init display
  Serial.println("[INIT] Resetting GT911 touch controller...");
  pinMode(23, OUTPUT);
  digitalWrite(23, LOW);
  delay(10);
  digitalWrite(23, HIGH);
  delay(50);

  // 2. SD Manager
  Serial.println("[INIT] Initializing SD Manager...");
  if (!sd_manager_init()) {
    Serial.println("[INIT] SD Manager failed, continuing without SD");
  }

  // 3. Configuration
  Serial.println("[INIT] Loading configuration...");
  if (config_load()) {
    Serial.println("[INIT] Configuration loaded successfully");
  }

  // 4. Logger
  Serial.println("[INIT] Initializing logger...");
  logger_init();

  // 5. Memory monitor
  Serial.println("[INIT] Initializing memory monitor...");
  memory_monitor_init();

  // 6. Display Board (ESP32_Display_Panel)
  Serial.println("[INIT] Initializing Display Board...");
  if (!display_init_board()) {
    Serial.println("[FATAL] Display Board init failed!");
    while (1) delay(1000);
  }

  // 7. LVGL
  Serial.println("[INIT] Initializing LVGL...");
  if (!display_init_lvgl()) {
    Serial.println("[FATAL] LVGL init failed!");
    while (1) delay(1000);
  }

  // 8. Touch
  Serial.println("[INIT] Initializing Touch...");
  if (!display_init_touch()) {
    Serial.println("[WARNING] Touch init failed");
  }

  // 9. I2C Wire1 (capteurs)
  Serial.println("[INIT] Initializing Wire1 (sensors)...");
  Wire1.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);
  Wire1.setTimeout(100);  // 100ms timeout
  Serial.println("[INIT] Wire1 initialized");

  // 10. Scan I2C
  sensor_scan_i2c();

  // 11. Capteurs
  Serial.println("[INIT] Initializing sensors...");
  if (!sensor_init_all()) {
    Serial.println("[WARNING] Sensor init had errors");
  }
  sensor_print_summary();

  // 12. USB MSC
  Serial.println("[INIT] Initializing USB MSC...");
  if (!usb_msc_init()) {
    Serial.println("[WARNING] USB MSC init failed");
  }

  // 13. UI simple
  Serial.println("[INIT] Creating UI...");
  lv_obj_t* scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

  lv_obj_t* label_title = lv_label_create(scr);
  lv_label_set_text(label_title, "VARIOMETER READY");
  lv_obj_set_style_text_font(label_title, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(label_title, lv_color_hex(0x00FF00), 0);
  lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 20);

  label_flight = lv_label_create(scr);
  lv_label_set_text(label_flight, "Waiting for sensors...");
  lv_obj_set_style_text_font(label_flight, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(label_flight, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(label_flight, LV_ALIGN_CENTER, 0, 0);

  lv_screen_load(scr);

  // 14. Task Flight
  Serial.println("[INIT] Starting task flight...");
  if (!task_flight_start()) {
    Serial.println("[FATAL] Task flight start failed!");
    while (1) delay(1000);
  }

  Serial.println("===========================================");
  Serial.println("  READY - ALL SYSTEMS OPERATIONAL");
  Serial.println("===========================================");
}

void loop() {
  static uint32_t last_print = 0;

  // Handler LVGL
  display_task();

  // Test périodique capteurs (5s)
  if (millis() - last_print > 5000) {
    last_print = millis();

    LOG_I(LOG_MODULE_SYSTEM, "=== Status ===");

    // BMP5
    if (sensor_bmp5_ready) {
      sensor_read_bmp5();  // ← Utiliser sensor_read_bmp5()
      LOG_I(LOG_MODULE_BMP5, "T=%.1f°C P=%.0fhPa Alt=%.0fm",
            g_bmp5_data.temperature, g_bmp5_data.pressure, g_bmp5_data.altitude);
    }

    // GPS
    if (sensor_gps_ready && g_gps_data.fix) {  // ← Utiliser g_gps_data
      LOG_I(LOG_MODULE_GPS, "Fix=%d Sats=%d Alt=%.0fm",
            g_gps_data.fix, g_gps_data.satellites, g_gps_data.altitude);
    }

    // Battery (si implémenté)
    if (sensor_battery_ready) {
      sensor_read_battery();
      LOG_I(LOG_MODULE_SYSTEM, "Bat: %.0f%% (%.2fV)", 
            g_battery_data.soc, g_battery_data.voltage);
    }
  }

  delay(5);
}