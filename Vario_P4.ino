/**
 * @file Vario_P4.ino
 * @brief Point d'entrée du variomètre ESP32-P4
 */

#include <Arduino.h>
#include "config/config.h"
#include "config/pins.h"
#include "src/hal/display_init.h"
#include "src/system/sd_manager/sd_manager.h"
#include "src/system/config_loader/config_loader.h"
#include "src/system/logger/logger.h"
#include "src/system/memory_monitor/memory_monitor.h"
#include "src/hal/i2c_wrapper/i2c_wrapper.h"
#include "src/data/config_data.h"

// Variable globale de configuration (définie dans config_data.h)
variometer_config_t g_config = { 0 };

void scanI2C(i2c_bus_id_t bus) {
  Serial.println("Scan I2C en cours...");
  uint8_t found = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    if (i2c_probe_device(bus, addr)) 
    {
      Serial.printf("✅ Périphérique détecté à l'adresse 0x%02X\n", addr);
      found++;
    }
  }

  if (found == 0)
    Serial.println("❌ Aucun périphérique I2C détecté");
  else
    Serial.printf("Total périphériques trouvés : %d\n", found);

  Serial.println("-----------------------------");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=================================");
  Serial.println("  Variometer ESP32-P4 Starting");
  Serial.println("=================================");

  // 1. Initialiser SD Manager (init SD_MMC + création dossiers système)
  Serial.println("[INIT] Initializing SD Manager...");
  if (!sd_manager_init()) {
    Serial.println("[INIT] SD Manager failed, continuing without SD");
  }

  // 2. Charger la configuration (init LittleFS automatiquement si besoin)
  Serial.println("[INIT] Loading configuration...");
  if (config_load()) {
    Serial.println("[INIT] Configuration loaded successfully");

    // Afficher la source de configuration
    switch (g_config.config_source) {
      case CONFIG_SOURCE_SD:
        Serial.println("[INIT] Using configuration from SD card");
        break;

      case CONFIG_SOURCE_LITTLEFS:
        Serial.println("[INIT] Using configuration from LittleFS (flash)");
        break;

      case CONFIG_SOURCE_HARDCODED:
        Serial.println("[INIT] Using hardcoded default configuration");
        break;
    }
  } else {
    Serial.println("[INIT] Configuration load failed, using defaults");
  }

  // 3. Initialiser le système de logging
  Serial.println("[INIT] Initializing logger...");
  if (!logger_init()) {
    Serial.println("[INIT] Logger initialization failed");
  }

  // 4. Initialiser le monitoring mémoire
  Serial.println("[INIT] Initializing memory monitor...");
  memory_monitor_init();

  //Init SD
  sd_manager_init();

  // Init Board (Display + Touch)
  if (!display_init_board()) {
    Serial.println("[FATAL] Board init failed!");
    while (1) delay(1000);
  }

  // Init LVGL
  if (!display_init_lvgl()) {
    Serial.println("[FATAL] LVGL init failed!");
    while (1) delay(1000);
  }

  // Créer UI test
  lv_obj_t* scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

  lv_obj_t* label = lv_label_create(scr);
  lv_label_set_text(label, "VARIOMETER READY!");
  lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(label, lv_color_hex(0x00FF00), 0);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

  lv_screen_load(scr);
  lv_obj_update_layout(scr);
  lv_refr_now(g_display);

  Serial.println("===========================================");
  Serial.println("  READY");
  Serial.println("===========================================");

  i2c_bus_config_t cfg = {
    .sda_pin = I2C_SDA_PIN,
    .scl_pin = I2C_SCL_PIN,
    .frequency = I2C_FREQUENCY
  };

  i2c_init(I2C_PORT, &cfg);
  scanI2C(I2C_PORT);
}

void loop() {
  display_task();
  static unsigned long last_print = 0;
  if (millis() - last_print > 30000) {
    last_print = millis();
    LOG_I(LOG_MODULE_SYSTEM, "=== Heartbeat ===");

    // Rapport mémoire complet
    memory_monitor_print_report();

    // Stats SD si disponible
    if (sd_manager_is_available()) {
      uint64_t free_space = sd_manager_free_space() / (1024 * 1024);
      uint64_t total_space = sd_manager_total_space() / (1024 * 1024);
      LOG_I(LOG_MODULE_STORAGE, "SD card: %llu/%llu MB free", free_space, total_space);
    }
  }
  delay(1);
}