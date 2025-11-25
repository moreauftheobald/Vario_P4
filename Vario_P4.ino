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
    delay(2000);
    // Scanner les réseaux
    //WiFiNetwork networks[10];
    //int count = wifi_scan(networks, 10);

    //if (count > 0) {
    //  LOG_I(LOG_WIFI, "Networks found:");
    //  for (int i = 0; i < count; i++) {
    //    LOG_I(LOG_WIFI, "  %s (RSSI: %d, %s)",
    //          networks[i].ssid,
    //          networks[i].rssi,
    //          wifi_encryption_type_str(networks[i].encryption));
    //  }

    // Tenter connexion si credentials configurés
    // if (strlen(wifi_ssid) > 0 && strcmp(wifi_ssid, "YOUR_SSID") != 0) {
    const char *ssid = "NAWAK";           // Change this to your WiFi SSID
    const char *password = "1234567890";  // Change this to your WiFi password
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println();
    wifi_print_info();

    //if (wifi_connect(WIFI_DEFAULT_SSID, WIFI_DEFAULT_PASSWORD)) {
    //  wifi_print_info();
    //}
    //  }
    //}
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