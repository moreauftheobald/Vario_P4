/**
 * @file Vario_P4.ino
 * @brief Point d'entrée principal du variomètre ESP32-P4
 * 
 * @author Franck Moreau
 * @date 2025-11-18
 * @version 1.0
 */

#include "config/config.h"
#include "config/pins.h"
#include "src/hal/display_init.h"
#include "src/system/sd_manager/sd_manager.h"
#include "src/system/config_loader/config_loader.h"
#include "src/system/logger/logger.h"
#include "src/system/memory_monitor/memory_monitor.h"
#include "src/system/sensor_init/sensor_init.h"
#include "src/system/imu_calibration/imu_calibration.h"
#include "src/tasks/task_flight.h"
#include "src/data/config_data.h"

// Variable globale de configuration (définie dans config_data.h)
variometer_config_t g_config = { 0 };

void setup() {
  Serial.begin(115200);
  delay(DELAY_STARTUP_MS);

  Serial.println("===========================================");
  Serial.println("  " PROJECT_NAME " v" PROJECT_VERSION);
  Serial.println("  Hardware: " HARDWARE_VERSION);
  Serial.println("  Build: " COMPILATION_DATE " " COMPILATION_TIME);
  Serial.println("===========================================");
  Serial.println();

  // =========================================================================
  // 1. INITIALISATION SD MANAGER
  // =========================================================================
  Serial.println("[INIT] Step 1/8: SD Manager");
  if (!sd_manager_init()) {
    Serial.println("[INIT] ⚠ SD Manager failed - continuing without SD");
  } else {
    Serial.println("[INIT] ✓ SD Manager initialized");
  }
  Serial.println();

  // =========================================================================
  // 2. CHARGEMENT CONFIGURATION
  // =========================================================================
  Serial.println("[INIT] Step 2/8: Configuration");
  if (config_load()) {
    Serial.println("[INIT] ✓ Configuration loaded");

    // Afficher la source de configuration
    switch (g_config.config_source) {
      case CONFIG_SOURCE_SD:
        Serial.println("[INIT]   Source: SD card");
        break;
      case CONFIG_SOURCE_LITTLEFS:
        Serial.println("[INIT]   Source: LittleFS (flash)");
        break;
      case CONFIG_SOURCE_HARDCODED:
        Serial.println("[INIT]   Source: Hardcoded defaults");
        break;
    }
  } else {
    Serial.println("[INIT] ⚠ Configuration load failed, using defaults");
  }
  Serial.println();

  // =========================================================================
  // 3. INITIALISATION LOGGER
  // =========================================================================
  Serial.println("[INIT] Step 3/8: Logger");
  if (!logger_init()) {
    Serial.println("[INIT] ⚠ Logger initialization failed");
  } else {
    Serial.println("[INIT] ✓ Logger initialized");
  }
  Serial.println();

  // À partir d'ici, on peut utiliser les macros LOG_X()
  LOG_I(LOG_MODULE_SYSTEM, "=== System Initialization ===");

  // =========================================================================
  // 4. INITIALISATION ÉCRAN + LVGL
  // =========================================================================
  LOG_I(LOG_MODULE_SYSTEM, "Step 4/8: Display + LVGL");
  
  // Initialiser le panneau MIPI DSI
  if (!display_init_panel()) {
    LOG_E(LOG_MODULE_SYSTEM, "Display panel initialization FAILED!");
    Serial.println();
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  ERREUR ECRAN - DEMARRAGE IMPOSSIBLE  ║");
    Serial.println("║                                        ║");
    Serial.println("║  Verifier:                             ║");
    Serial.println("║  - PSRAM active (16MB requis)         ║");
    Serial.println("║  - Connexion ecran MIPI DSI            ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
    while (1) delay(1000);
  }
  LOG_I(LOG_MODULE_SYSTEM, "✓ Display panel initialized");
  
  // Initialiser LVGL
  if (!display_init_lvgl()) {
    LOG_E(LOG_MODULE_SYSTEM, "LVGL initialization FAILED!");
    while (1) delay(1000);
  }
  LOG_I(LOG_MODULE_SYSTEM, "✓ LVGL initialized");
  
  // Afficher écran de boot
  display_show_boot_screen();

  // =========================================================================
  // 5. INITIALISATION MEMORY MONITOR
  // =========================================================================
  LOG_I(LOG_MODULE_SYSTEM, "Step 5/8: Memory Monitor");
  if (!memory_monitor_init()) {
    LOG_W(LOG_MODULE_SYSTEM, "Memory monitor initialization failed");
  } else {
    LOG_I(LOG_MODULE_SYSTEM, "Memory monitor initialized");
  }

  // =========================================================================
  // 6. INITIALISATION CAPTEURS I2C
  // =========================================================================
  LOG_I(LOG_MODULE_SYSTEM, "Step 6/8: Sensors");
  if (!sensor_init_all()) {
    LOG_E(LOG_MODULE_SYSTEM, "Sensor initialization FAILED!");
    LOG_E(LOG_MODULE_SYSTEM, "Cannot continue without sensors");

    // Afficher message d'erreur et bloquer
    Serial.println();
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  ERREUR CAPTEURS - DEMARRAGE IMPOSSIBLE  ║");
    Serial.println("║                                          ║");
    Serial.println("║  Vérifier:                              ║");
    Serial.println("║  - Connexions I2C (SDA/SCL)            ║");
    Serial.println("║  - Alimentation capteurs                ║");
    Serial.println("║  - Adresses I2C correctes               ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();

    while (1) {
      delay(1000);
    }
  }

  // Vérifier que les capteurs critiques sont OK
  if (!sensor_check_critical()) {
    LOG_E(LOG_MODULE_SYSTEM, "Critical sensors missing!");
    Serial.println();
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  CAPTEURS CRITIQUES MANQUANTS          ║");
    Serial.println("║                                         ║");
    Serial.println("║  LSM6DSO32 et BMP585 requis            ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
    while (1) delay(1000);
  }

  LOG_I(LOG_MODULE_SYSTEM, "All critical sensors initialized");

  // =========================================================================
  // 7. CALIBRATION IMU
  // =========================================================================
  LOG_I(LOG_MODULE_SYSTEM, "Step 7/8: IMU Calibration");

  if (!imu_calibration_startup()) {
    LOG_E(LOG_MODULE_SYSTEM, "IMU calibration failed!");
    LOG_E(LOG_MODULE_SYSTEM, "Cannot continue without valid calibration");

    while (1) {
      delay(1000);
    }
  }

  LOG_I(LOG_MODULE_SYSTEM, "IMU calibration complete");

  // =========================================================================
  // 8. DEMARRAGE TACHES FREERTOS
  // =========================================================================
  LOG_I(LOG_MODULE_SYSTEM, "Step 8/8: Starting FreeRTOS tasks");

  // Tâche flight (capteurs + fusion + Kalman + calculs)
  if (!task_flight_init()) {
    LOG_E(LOG_MODULE_SYSTEM, "Failed to create flight task!");
    while (1) delay(1000);
  }
  LOG_I(LOG_MODULE_SYSTEM, "✓ Flight task started");

  // TODO: Autres tâches à créer
  // task_display_init();
  // task_storage_init();

  // =========================================================================
  // FIN INITIALISATION
  // =========================================================================
  LOG_I(LOG_MODULE_SYSTEM, "=== Initialization Complete ===");
  LOG_I(LOG_MODULE_SYSTEM, "");
  LOG_I(LOG_MODULE_SYSTEM, "Variometer ready for flight!");
  LOG_I(LOG_MODULE_SYSTEM, "");

  Serial.println();
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║                                        ║");
  Serial.println("║      VARIOMETRE PRET A VOLER !         ║");
  Serial.println("║                                        ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();

  // Afficher rapport mémoire initial
  memory_monitor_print_report();
}

void loop() {
  static unsigned long last_heartbeat = 0;
  static unsigned long last_lvgl_update = 0;
  
  // Mise à jour LVGL (~30 FPS)
  unsigned long now = millis();
  if (now - last_lvgl_update > 33) {  // ~30ms = 30 FPS
    display_lvgl_task();
    last_lvgl_update = now;
  }
  
  // Heartbeat toutes les 10 secondes
  if (now - last_heartbeat > 10000) {
    last_heartbeat = now;
    
    LOG_I(LOG_MODULE_SYSTEM, "=== Heartbeat ===");
    
    // Copier flight_data
    flight_data_t fd;
    if (task_flight_get_data(&fd)) {
      LOG_I(LOG_MODULE_SYSTEM, "Sensors health:");
      LOG_I(LOG_MODULE_SYSTEM, "  IMU   : %s (errors: %d)", 
            fd.sensors_health.imu_healthy ? "✓ OK" : "✗ FAIL",
            fd.sensors_health.imu_error_count);
      LOG_I(LOG_MODULE_SYSTEM, "  BARO  : %s (errors: %d)",
            fd.sensors_health.baro_healthy ? "✓ OK" : "✗ FAIL",
            fd.sensors_health.baro_error_count);
      LOG_I(LOG_MODULE_SYSTEM, "  GPS   : %s (fix: %d sats)",
            fd.sensors_health.gps_healthy ? "✓ OK" : "✗ NO FIX",
            fd.satellites);
      
      LOG_I(LOG_MODULE_SYSTEM, "Flight data:");
      LOG_I(LOG_MODULE_SYSTEM, "  Altitude: %.1f m", fd.altitude_qnh);
      LOG_I(LOG_MODULE_SYSTEM, "  Vario: %.2f m/s", fd.vario);
    }
  }
}