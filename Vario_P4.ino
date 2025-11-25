/**
 * @file Vario_P4.ino
 * @brief Variomètre ESP32-P4 - V2.0
 */

#include "src/config/config.h"
#include "src/config/pins.h"
#include "src/system/logger.h"
#include "src/hal/display.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  LOG_I(LOG_SYSTEM, "========================================");
  LOG_I(LOG_SYSTEM, "  %s v%s", PROJECT_NAME, PROJECT_VERSION);
  LOG_I(LOG_SYSTEM, "========================================");

  // Init Display Board
  if (!init_display_gloabl()) {
    LOG_E(LOG_SYSTEM, "FATAL: Display init failed");
    while (1) delay(1000);
  }

  // Créer écran de test
  display_create_test_screen();
  
  // Forcer un premier refresh
  lv_refr_now(display);
  delay(50);

  LOG_I(LOG_SYSTEM, "========================================");
  LOG_I(LOG_SYSTEM, "  READY!");
  LOG_I(LOG_SYSTEM, "========================================");
}

void loop() {
  display_task();
  delay(1);
}