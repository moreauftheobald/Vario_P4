#include "config/config.h"
#include "config/pins.h"
#include "src/system/config_loader/config_loader.h"

void setup() {
    Serial.begin(115200);
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
        while(file) {
            Serial.printf("  - %s (%d bytes)\n", file.name(), file.size());
            file = root.openNextFile();
        }
    }
    
    // 2. Initialiser SD_MMC (on verra après pour les erreurs)
    Serial.println("[INIT] Mounting SD_MMC...");
    if (!SD_MMC.begin("/sdcard", true)) {
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
        switch(cfg->config_source) {
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