/**
 * @file Vario_P4.ino
 * @brief Point d'entrée principal du variomètre ESP32-P4
 * 
 * @author Theobald Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#include "config/config.h"
#include "config/pins.h"
#include "src/system/sd_manager/sd_manager.h"
#include "src/system/config_loader/config_loader.h"
#include "src/system/logger/logger.h"
#include "src/system/memory_monitor/memory_monitor.h"
#include "src/data/config_data.h"

// Variable globale de configuration (définie dans config_data.h)
variometer_config_t g_config = {0};

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
    
    // Test du logger
    LOG_I(LOG_MODULE_SYSTEM, "Variometer initialization complete");
    LOG_V(LOG_MODULE_SYSTEM, "QNH: %.2f hPa", g_config.flight_params.qnh);
    LOG_V(LOG_MODULE_SYSTEM, "Vario damping: %.2f", g_config.flight_params.vario_damping);
    
    Serial.println("=================================");
    Serial.println("  Setup Complete");
    Serial.println("=================================");
}

void loop() {
    // Heartbeat toutes les 30 secondes avec monitoring mémoire
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
    
    delay(100);
}