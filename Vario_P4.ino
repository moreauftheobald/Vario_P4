/**
 * @file Vario_P4.ino
 * @brief Variomètre ESP32-P4 - V2.0
 */
#include <WiFi.h>
#include "src/config/config.h"
#include "src/config/pins.h"
#include "src/system/logger.h"
#include "src/hal/display.h"
#include "src/hal/sensor_init.h"
#include "src/hal/sd_helper.h"
#include "src/hal/wifi_helper.h"
#include "src/ui/splash_screen.h"
#include "src/ui/ui_prestart.h"

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

  // ========================================
  // SPLASH SCREEN (non-bloquant)
  // ========================================
  splash_screen_show(3000);

  // Init en parallèle pendant que le splash est affiché
  init_sensors_global(0);
  sd_init();

  WiFi.begin(wifi_ssid, wifi_password);

  // Attendre connexion OU fin du splash
  while (!splash_screen_should_close()) {
    lv_timer_handler();
    delay(5);

    // Si WiFi connecté, on peut sortir plus tôt
    if (WiFi.status() == WL_CONNECTED) {
      LOG_I(LOG_WIFI, "Connected! IP: %s", WiFi.localIP().toString().c_str());
    }
  }

  // Fermer le splash
  splash_screen_close();

  // Créer écran principal
  prestart_show();
  //display_create_test_screen();
  lv_refr_now(display);

  LOG_I(LOG_SYSTEM, "  READY!");
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