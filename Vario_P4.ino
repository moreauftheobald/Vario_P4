/**
 * @file Vario_P4.ino
 * @brief Point d'entrée du variomètre ESP32-P4 avec task_flight
 */

#include <Arduino.h>
#include "config/config.h"
#include "config/pins.h"
#include "src/hal/display_init.h"
#include "src/system/sd_manager/sd_manager.h"
#include "src/system/config_loader/config_loader.h"
#include "src/system/logger/logger.h"
#include "src/system/memory_monitor/memory_monitor.h"
#include "src/system/usb_msc_manager/usb_msc_manager.h"
#include "src/hal/i2c_wrapper/i2c_wrapper.h"
#include "src/system/sensor_init/sensor_init.h"
#include "src/data/config_data.h"
#include "src/system/BMP5XX_ESP32/BMP5XX_ESP32.h"
#include "src/tasks/task_flight.h"

// Variable globale de configuration
variometer_config_t g_config = { 0 };

// Variable globale des données de vol
flight_data_t g_flight_data = { 0 };

// Variables UI globales
lv_obj_t* label_flight = nullptr;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=================================");
  Serial.println("  Variometer ESP32-P4 Starting");
  Serial.println("=================================");

  // ✅ Reset GT911 AVANT tout
  Serial.println("[INIT] Resetting touch controller...");
  pinMode(23, OUTPUT);
  digitalWrite(23, LOW);
  delay(10);
  digitalWrite(23, HIGH);
  delay(50);

  // 1. Initialiser SD Manager
  Serial.println("[INIT] Initializing SD Manager...");
  if (!sd_manager_init()) {
    Serial.println("[INIT] SD Manager failed, continuing without SD");
  }

  // 2. Charger configuration
  Serial.println("[INIT] Loading configuration...");
  if (config_load()) {
    Serial.println("[INIT] Configuration loaded successfully");
    switch (g_config.config_source) {
      case CONFIG_SOURCE_SD:
        Serial.println("[INIT] Using configuration from SD card");
        break;
      case CONFIG_SOURCE_LITTLEFS:
        Serial.println("[INIT] Using configuration from LittleFS");
        break;
      case CONFIG_SOURCE_HARDCODED:
        Serial.println("[INIT] Using hardcoded configuration");
        break;
    }
  } else {
    Serial.println("[INIT] Configuration load failed, using defaults");
  }

  // 3. Initialiser logger
  Serial.println("[INIT] Initializing logger...");
  if (!logger_init()) {
    Serial.println("[INIT] Logger initialization failed");
  }

  // 4. Initialiser memory monitor
  Serial.println("[INIT] Initializing memory monitor...");
  memory_monitor_init();

  // 5. Init Display (qui init I2C Bus 0 pour GT911)
  Serial.println("[INIT] Initializing Display...");
  if (!display_init_board()) {
    Serial.println("[FATAL] Board init failed!");
    while (1) delay(1000);
  }

  // 6. Init LVGL
  Serial.println("[INIT] Initializing LVGL...");
  if (!display_init_lvgl()) {
    Serial.println("[FATAL] LVGL init failed!");
    while (1) delay(1000);
  }

  // 7. Init I2C Bus 1 (capteurs) APRÈS Display
  Serial.println("[INIT] Initializing I2C Bus 1 (sensors)...");
  i2c_bus_config_t cfg = {
    .sda_pin = I2C_SDA_PIN,
    .scl_pin = I2C_SCL_PIN,
    .frequency = I2C_FREQUENCY,
    .enabled = true
  };

  if (!i2c_init(I2C_BUS_1, &cfg)) {
    Serial.println("[FATAL] I2C Bus 1 init failed!");
    while (1) delay(1000);
  }
  Serial.println("[INIT] I2C Bus 1 initialized");

  // 8. Initialiser capteurs
  Serial.println("[INIT] Initializing sensors...");
  if (!sensor_init_all()) {
    Serial.println("[WARNING] Sensor init had errors");
  }

  // 9. Initialiser USB MSC
  Serial.println("[INIT] Initializing USB MSC...");
  if (!usb_msc_init()) {
    Serial.println("[WARNING] USB MSC init failed");
  }

  // 10. Créer UI simple
  Serial.println("[INIT] Creating UI...");

  lv_obj_t* scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

  // Titre
  lv_obj_t* label_title = lv_label_create(scr);
  lv_label_set_text(label_title, "FLIGHT TASK RUNNING");
  lv_obj_set_style_text_font(label_title, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(label_title, lv_color_hex(0x00FF00), 0);
  lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 20);

  // Label flight data (variable globale)
  label_flight = lv_label_create(scr);
  lv_label_set_text(label_flight, "Waiting for data...");
  lv_obj_set_style_text_font(label_flight, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(label_flight, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(label_flight, LV_ALIGN_CENTER, 0, 0);

  lv_screen_load(scr);
  lv_obj_update_layout(scr);
  lv_refr_now(g_display);

  // 11. DÉMARRER TASK FLIGHT
  Serial.println("[INIT] Starting task flight...");
  if (!task_flight_start()) {
    Serial.println("[FATAL] Task flight start failed!");
    while (1) delay(1000);
  }

  Serial.println("===========================================");
  Serial.println("  READY - TASK FLIGHT ACTIVE");
  Serial.println("===========================================");
}

void loop() {
  static uint32_t last_print = 0;

  // Test périodique capteurs (toutes les 5s)
  if (millis() - last_print > 5000) {
    last_print = millis();

    LOG_I(LOG_MODULE_SYSTEM, "");
    LOG_I(LOG_MODULE_SYSTEM, "=== System Status ===");

    // IMU
    /*if (sensor_imu_ready) {
#if IMU_BNO08XX == 1
      sh2_SensorValue_t event;
      if (bno08x_dev && bno08x_dev->getSensorEvent(&event)) {
        if (event.sensorId == SH2_LINEAR_ACCELERATION) {
          LOG_I(LOG_MODULE_IMU, "BNO08x Accel: X=%.3f Y=%.3f Z=%.3f m/s²",
                event.un.linearAcceleration.x,
                event.un.linearAcceleration.y,
                event.un.linearAcceleration.z);
        }
      }
#else
      lsm6dso32_data_t imu_data;
      if (LSM6DSO32_read(&lsm6dso32_dev, &imu_data)) {
        LOG_I(LOG_MODULE_IMU, "LSM6DSO32 Accel: X=%.3f Y=%.3f Z=%.3f m/s²",
              imu_data.accel_x, imu_data.accel_y, imu_data.accel_z);
        LOG_I(LOG_MODULE_IMU, "LSM6DSO32 Gyro: X=%.3f Y=%.3f Z=%.3f rad/s",
              imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
      }
#endif
    }*/

    // BMP5
    if (sensor_bmp5_ready) {
      bmp5_data_t bmp_data;
      if (BMP5_read(&bmp5_dev, &bmp_data, 1013.25f)) {
        LOG_I(LOG_MODULE_BMP5, "BMP5: T=%.2f°C P=%.2fhPa Alt=%.1fm",
              bmp_data.temperature, bmp_data.pressure, bmp_data.altitude);
      }
    }

    // GPS
    if (sensor_gps_ready && gps_data.fix) {
      LOG_I(LOG_MODULE_GPS, "GPS: Fix=%d Sats=%d Alt=%.1fm",
            gps_data.fix, gps_data.satellites, gps_data.altitude);
    }

    // Battery
    if (sensor_battery_ready) {
      max17048_data_t bat_data;
      if (MAX17048_read(&max17048_dev, &bat_data)) {
        LOG_I(LOG_MODULE_SYSTEM, "Battery: %.1f%% (%.2fV)",
              bat_data.soc, bat_data.voltage);
      }
    }

    LOG_I(LOG_MODULE_SYSTEM, "====================");
  }

  delay(100);
}
