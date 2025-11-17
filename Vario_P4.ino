/**
 * @file Vario_P4.ino
 * @brief Point d'entrée principal du variomètre ESP32-P4
 * 
 * @author Franck Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#include "config/config.h"
#include "config/pins.h"
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
  Serial.println("[INIT] Step 1/7: SD Manager");
  if (!sd_manager_init()) {
    Serial.println("[INIT] ⚠ SD Manager failed - continuing without SD");
  } else {
    Serial.println("[INIT] ✓ SD Manager initialized");
  }
  Serial.println();

  // =========================================================================
  // 2. CHARGEMENT CONFIGURATION
  // =========================================================================
  Serial.println("[INIT] Step 2/7: Configuration");
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
  Serial.println("[INIT] Step 3/7: Logger");
  if (!logger_init()) {
    Serial.println("[INIT] ⚠ Logger initialization failed");
  } else {
    Serial.println("[INIT] ✓ Logger initialized");
  }
  Serial.println();

  // À partir d'ici, on peut utiliser les macros LOG_X()
  LOG_I(LOG_MODULE_SYSTEM, "=== System Initialization ===");

  // =========================================================================
  // 4. INITIALISATION MEMORY MONITOR
  // =========================================================================
  LOG_I(LOG_MODULE_SYSTEM, "Step 4/7: Memory Monitor");
  if (!memory_monitor_init()) {
    LOG_W(LOG_MODULE_SYSTEM, "Memory monitor initialization failed");
  } else {
    LOG_I(LOG_MODULE_SYSTEM, "Memory monitor initialized");
  }

  // =========================================================================
  // 5. INITIALISATION CAPTEURS I2C
  // =========================================================================
  LOG_I(LOG_MODULE_SYSTEM, "Step 5/7: Sensors");
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
    Serial.println("║  LSM6DSO32 et BMP390 requis            ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
    while (1) delay(1000);
  }

  LOG_I(LOG_MODULE_SYSTEM, "All critical sensors initialized");

  LOG_I(LOG_MODULE_SYSTEM, "Step 6/7: IMU Calibration");

  if (!imu_calibration_startup()) {
    LOG_E(LOG_MODULE_SYSTEM, "IMU calibration failed!");
    LOG_E(LOG_MODULE_SYSTEM, "Cannot continue without valid calibration");

    while (1) {
      delay(1000);
    }
  }

  LOG_I(LOG_MODULE_SYSTEM, "IMU calibration complete");

  // =========================================================================
  // 7. DEMARRAGE TACHES FREERTOS
  // =========================================================================
  LOG_I(LOG_MODULE_SYSTEM, "Step 7/7: Starting FreeRTOS tasks");

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
  // Heartbeat monitoring toutes les 30 secondes
  static unsigned long last_heartbeat = 0;

  if (millis() - last_heartbeat > 10000) {
    last_heartbeat = millis();

    LOG_I(LOG_MODULE_SYSTEM, "");
    LOG_I(LOG_MODULE_SYSTEM, "=== Heartbeat ===");

    // Rapport mémoire
    memory_monitor_print_report();

    // Statut capteurs
    LOG_I(LOG_MODULE_SYSTEM, "Sensors status:");
    LOG_I(LOG_MODULE_SYSTEM, "  LSM6DSO32 (IMU)  : %s",
          sensor_lsm6dso32_ready ? "✓ OK" : "✗ FAIL");
    LOG_I(LOG_MODULE_SYSTEM, "  BMP390 (Baro)    : %s",
          sensor_bmp390_ready ? "✓ OK" : "✗ FAIL");
    LOG_I(LOG_MODULE_SYSTEM, "  PA1010D (GPS)    : %s",
          sensor_gps_ready ? "✓ OK" : "✗ FAIL");

    // Stats SD si disponible
    if (sd_manager_is_available()) {
      uint64_t free_space = sd_manager_free_space() / (1024 * 1024);
      uint64_t total_space = sd_manager_total_space() / (1024 * 1024);
      LOG_I(LOG_MODULE_STORAGE, "SD card: %llu/%llu MB free (%.1f%%)",
            free_space, total_space,
            (free_space * 100.0f) / total_space);
    }

    // Données de vol (exemple)
    flight_data_t data;
    if (task_flight_get_data(&data)) {
      LOG_I(LOG_MODULE_FLIGHT, "Flight data:");
      LOG_I(LOG_MODULE_FLIGHT, "  Altitude: %.1f m", data.altitude_qnh);
      LOG_I(LOG_MODULE_FLIGHT, "  Vario: %.2f m/s", data.vario);
      LOG_I(LOG_MODULE_FLIGHT, "  GPS: %s (%d sats)",
            data.gps_fix ? "FIX" : "NO FIX", data.satellites);
      LOG_I(LOG_MODULE_FLIGHT, "  In flight: %s",
            data.in_flight ? "YES" : "NO");

      if (data.in_flight) {
        LOG_I(LOG_MODULE_FLIGHT, "  Flight time: %02d:%02d:%02d",
              data.time_flight / 3600,
              (data.time_flight % 3600) / 60,
              data.time_flight % 60);
      }
    }

    LOG_I(LOG_MODULE_SYSTEM, "");
  }

  delay(100);
}