/**
 * @file Vario_P4.ino
 * @brief Variomètre ESP32-P4 - V2.0
 */

#include "src/config/config.h"
#include "src/config/pins.h"
#include "src/system/logger.h"
#include "src/hal/display.h"
#include "src/hal/sensor_init.h"
#include "src/hal/sd_helper.h"
#include "src/hal/wifi_helper.h"

void setup() {
  Serial.begin(115200);
  delay(1000);

  LOG_I(LOG_SYSTEM, "========================================");
  LOG_I(LOG_SYSTEM, "  %s v%s", PROJECT_NAME, PROJECT_VERSION);
  LOG_I(LOG_SYSTEM, "========================================");

  // Init Display
  if (!init_display_gloabl()) {
    LOG_E(LOG_SYSTEM, "FATAL: Display init failed");
    while (1) delay(1000);
  }

  // Init Sensors
  if (!init_sensors_global(0)) {
    LOG_E(LOG_SYSTEM, "FATAL: Sensors init failed");
    while (1) delay(1000);
  }

  // Init SD
  if (!sd_init()) {
    LOG_W(LOG_SYSTEM, "SD init failed, continuing...");
  }

  // Init WiFi
  if (wifi_init()) {
  LOG_I(LOG_WIFI, "WiFi initialized");
  
  // Connexion directe si credentials configurés
  if (strcmp(wifi_ssid, "YOUR_SSID") != 0) {
    if (wifi_connect(wifi_ssid, wifi_password)) {
      wifi_print_info();
    } else {
      LOG_W(LOG_WIFI, "Connection failed, continuing without WiFi...");
    }
  } else {
    LOG_W(LOG_WIFI, "No credentials configured, WiFi disabled");
  }
}

  // Créer écran de test
  display_create_test_screen();
  lv_refr_now(display);

  LOG_I(LOG_SYSTEM, "========================================");
  LOG_I(LOG_SYSTEM, "  READY!");
  LOG_I(LOG_SYSTEM, "========================================");
}

void loop() {
  static bool first_run = true;
  if (first_run) {
    LOG_I(LOG_SYSTEM, "Loop running on core %d", xPortGetCoreID());
    first_run = false;
  }
  display_task();
  delay(1);
}