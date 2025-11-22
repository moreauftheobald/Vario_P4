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
  // 1. Tâche LVGL
  display_task();

  // ✅ TEST DEBUG IMU (temporaire)
  static unsigned long last_imu_debug = 0;
  unsigned long now = millis();

  if (now - last_imu_debug >= 1000) {
    last_imu_debug = now;

    // Lire IMU
    lsm6dso32_data_t imu_data;
    if (LSM6DSO32_read(&lsm6dso32_dev, &imu_data)) {
      Serial.println("=== MADGWICK DETAILED DEBUG ===");

      // 1. Données brutes
      float norm = sqrtf(imu_data.accel_x * imu_data.accel_x + imu_data.accel_y * imu_data.accel_y + imu_data.accel_z * imu_data.accel_z);
      Serial.printf("Accel RAW: X=%.3f Y=%.3f Z=%.3f (norm=%.3f)\n",
                    imu_data.accel_x, imu_data.accel_y, imu_data.accel_z, norm);
      Serial.printf("Gyro RAW:  X=%.3f Y=%.3f Z=%.3f rad/s\n",
                    imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);

      float gyro_x_cal = imu_data.gyro_x - gyro_offset_x;
      float gyro_y_cal = imu_data.gyro_y - gyro_offset_y;
      float gyro_z_cal = imu_data.gyro_z - gyro_offset_z;
      Serial.printf("Gyro CAL:  X=%.6f Y=%.6f Z=%.6f rad/s (after offset)\n",
                    gyro_x_cal, gyro_y_cal, gyro_z_cal);

      // 2. Quaternion
      Serial.printf("Quaternion: w=%.4f x=%.4f y=%.4f z=%.4f\n",
                    madgwick.q.w, madgwick.q.x, madgwick.q.y, madgwick.q.z);

      float q_norm = sqrtf(madgwick.q.w * madgwick.q.w + madgwick.q.x * madgwick.q.x + madgwick.q.y * madgwick.q.y + madgwick.q.z * madgwick.q.z);
      Serial.printf("Quat norm: %.6f (should be 1.0)\n", q_norm);

      // ✅ 3. TEST QUATERNION NORMAL vs INVERSE
      Serial.println("--- Transform Test ---");

      // Q normal
      quaternion_t q_normal = madgwick.q;
      float ex1, ey1, ez1;
      madgwick_rotate_vector_test(&q_normal,
                                  imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                                  &ex1, &ey1, &ez1);
      Serial.printf("Earth (Q normal):  X=%.3f Y=%.3f Z=%.3f\n", ex1, ey1, ez1);

      // Q inverse (conjugué)
      quaternion_t q_inv = madgwick.q;
      q_inv.x = -q_inv.x;
      q_inv.y = -q_inv.y;
      q_inv.z = -q_inv.z;
      float ex2, ey2, ez2;
      madgwick_rotate_vector_test(&q_inv,
                                  imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                                  &ex2, &ey2, &ez2);
      Serial.printf("Earth (Q inverse): X=%.3f Y=%.3f Z=%.3f\n", ex2, ey2, ez2);

      Serial.println("One should have Z≈9.81, X≈0, Y≈0");
      Serial.println("----------------------");

      // 4. Transformation actuelle (via fonction existante)
      vector3_t earth_accel;
      madgwick_get_earth_accel(&madgwick,
                               imu_data.accel_x,
                               imu_data.accel_y,
                               imu_data.accel_z,
                               &earth_accel);

      Serial.printf("Current function:  X=%.3f Y=%.3f Z=%.3f m/s²\n",
                    earth_accel.x, earth_accel.y, earth_accel.z);

      float earth_norm = sqrtf(earth_accel.x * earth_accel.x + earth_accel.y * earth_accel.y + earth_accel.z * earth_accel.z);
      Serial.printf("Earth norm: %.3f (should be ~9.81)\n", earth_norm);

      // 5. Accélération verticale
      float accel_vert = earth_accel.z - 9.81f;
      Serial.printf("Vertical accel: %.3f m/s² (expected ~0.0)\n", accel_vert);

      // 6. Angles d'Euler
      euler_t euler;
      madgwick_quaternion_to_euler(&madgwick.q, &euler);
      Serial.printf("Euler: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°\n",
                    euler.roll, euler.pitch, euler.yaw);

      Serial.println("================================");
    }
  }

  // 2. Mise à jour affichage
  static unsigned long last_display_update = 0;
  if (now - last_display_update >= 500) {
    last_display_update = now;

    flight_data_t flight_data;
    if (task_flight_get_data(&flight_data)) {
      char buf[128];
      snprintf(buf, sizeof(buf),
               "Alt: %.1f m\nVario: %.2f m/s\nSpeed: %.1f km/h\nGPS: %d sats%s",
               flight_data.altitude_qnh,
               flight_data.vario,
               flight_data.speed_ground,
               flight_data.satellites,
               flight_data.gps_fix ? " FIX" : "");

      lv_label_set_text(label_flight, buf);
    }
  }

  delay(1);
}