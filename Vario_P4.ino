#include "src/system/config_loader/config_loader.h"
#include "config/pins.h"
#include <SD_MMC.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=================================");
  Serial.println("  Variometer ESP32-P4 Starting");
  Serial.println("=================================");

  // 1. Initialiser SPIFFS
  Serial.println("[INIT] Mounting SPIFFS...");
  if (!SPIFFS.begin(true)) {  // true = format si necessaire
    Serial.println("[INIT] SPIFFS mount failed!");
  } else {
    Serial.println("[INIT] SPIFFS mounted successfully");

    // Debug: Lister les fichiers SPIFFS
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    Serial.println("[INIT] Files in SPIFFS:");
    while (file) {
      Serial.printf("  - %s (%d bytes)\n", file.name(), file.size());
      file = root.openNextFile();
    }
  }

  // 2. Initialiser SD_MMC
  Serial.println("[INIT] Mounting SD_MMC...");
  // Configurer les pins SDMMC
  if (!SD_MMC.setPins(SD_PIN_CLK, SD_PIN_CMD, SD_PIN_D0)) {
    Serial.println("[INIT] SD: Echec configuration pins");
  }

  delay(500);

  Serial.println("[INIT] SD: Pins configurees");
  if (!SD_MMC.begin("/sdcard", true)) {  // true = mode 1-bit
    Serial.println("[INIT] SD_MMC mount failed!");
  } else {
    Serial.println("[INIT] SD_MMC mounted successfully");
    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("[INIT] SD Card Size: %lluMB\n", cardSize);
  }

  // 3. Charger la configuration
  Serial.println("[INIT] Loading configuration...");
  if (config_init()) {
    Serial.println("[INIT] Configuration loaded successfully");

    // Afficher la source
    variometer_config_t* cfg = config_get();
    switch (cfg->config_source) {
      case CONFIG_SOURCE_SD:
        Serial.println("[INIT] Using SD config");
        break;
      case CONFIG_SOURCE_SPIFFS:
        Serial.println("[INIT] Using SPIFFS config");
        break;
      case CONFIG_SOURCE_HARDCODED:
        Serial.println("[INIT] Using hardcoded config");
        break;
    }
  } else {
    Serial.println("[INIT] Configuration loading failed!");
  }

  Serial.println("=================================");
  Serial.println("  Initialization Complete");
  Serial.println("=================================");
}

void loop() {
  delay(1000);
}