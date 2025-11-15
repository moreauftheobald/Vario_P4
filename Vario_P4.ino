#include "config/config.h"
#include "config/pins.h"
#include "src/system/config_loader/config_loader.h"

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(1000);

  Serial.println("=================================");
  Serial.println("  Variometer ESP32-P4 Starting");
  Serial.println("=================================");

  // 1. Initialiser LittleFS (pas SPIFFS !)
  Serial.println("[INIT] Mounting LittleFS...");
  if (!LittleFS.begin(true)) {  // true = format si necessaire
    Serial.println("[INIT] LittleFS mount failed!");
  } else {
    Serial.println("[INIT] LittleFS mounted successfully");

    // Debug: Lister les fichiers LittleFS
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    Serial.println("[INIT] Files in LittleFS:");
    while (file) {
      Serial.printf("  - %s (%d bytes)\n", file.name(), file.size());
      file = root.openNextFile();
    }
  }

  // 2. Initialiser SD_MMC avec les pins correctes
  Serial.println("[INIT] Mounting SD_MMC...");
  Serial.println("[INIT] SD: Configuring pins...");

  // Configurer les pins avant begin()
  if (!SD_MMC.setPins(SD_PIN_CLK, SD_PIN_CMD, SD_PIN_D0)) {
    Serial.println("[INIT] SD: Failed to set pins!");
  }

  // Initialiser en mode 1-bit
  if (!SD_MMC.begin("/sdcard", true)) {  // true = mode 1-bit
    Serial.println("[INIT] SD_MMC mount failed!");
    Serial.printf("[INIT] SD Error code: 0x%x\n", esp_err_t(SD_MMC.cardType()));
  } else {
    Serial.println("[INIT] SD_MMC mounted successfully!");

    if (SD_MMC.cardType() != CARD_NONE) {
      uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
      uint64_t cardUsed = SD_MMC.usedBytes() / (1024 * 1024);
      Serial.printf("[INIT] SD Card Size: %llu MB\n", cardSize);
      Serial.printf("[INIT] SD Card Used: %llu MB\n", cardUsed);
      Serial.printf("[INIT] SD Card Type: %d\n", SD_MMC.cardType());
    } else {
      Serial.println("[INIT] No SD card detected");
    }
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
      case CONFIG_SOURCE_LITTLEFS:
        Serial.println("[INIT] Using LittleFS config");
        break;
      case CONFIG_SOURCE_HARDCODED:
        Serial.println("[INIT] Using hardcoded config");
        break;
    }
  }

  Serial.println("=================================");
}

void loop() {
  delay(1000);
}