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

// Variables globales
variometer_config_t g_config = { 0 };
flight_data_t g_flight_data = { 0 };
lv_obj_t* label_flight = nullptr;

void test_bno080_deep() {
    Serial.println("\n=== TEST APPROFONDI BNO080 ===");
    
    // 1. Test avec différentes vitesses I2C
    test_bno080_speeds();
    
    // 2. Test du protocole SHTP
    test_bno080_shtp();
    
    // 3. Test de polling intensif
    test_bno080_polling();
}

void test_bno080_speeds() {
    Serial.println("\n1. Test avec différentes vitesses I2C:");
    
    uint32_t speeds[] = {10000, 50000, 100000, 400000};
    const char* names[] = {"10kHz", "50kHz", "100kHz", "400kHz"};
    
    for (int s = 0; s < 4; s++) {
        Serial.printf("\nTesting at %s:\n", names[s]);
        
        // Reconfigurer I2C
        i2c_driver_delete(I2C_NUM_1);
        delay(100);
        
        i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = (gpio_num_t)50;
        conf.scl_io_num = (gpio_num_t)49;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = speeds[s];
        
        i2c_param_config(I2C_NUM_1, &conf);
        i2c_driver_install(I2C_NUM_1, conf.mode, 0, 0, 0);
        i2c_set_timeout(I2C_NUM_1, 16000000);
        
        // Test lecture
        uint8_t data[4];
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (0x4A << 1) | I2C_MASTER_READ, false);
        i2c_master_read(cmd, data, 3, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data[3], I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(200));
        i2c_cmd_link_delete(cmd);
        
        Serial.printf("  Result: %s", esp_err_to_name(ret));
        if (ret == ESP_OK) {
            Serial.printf(" - Data: %02X %02X %02X %02X", 
                          data[0], data[1], data[2], data[3]);
            
            // Interpréter si c'est un header SHTP valide
            uint16_t len = data[0] | (data[1] << 8);
            len &= 0x7FFF;
            if (len > 0 && len < 300) {
                Serial.printf(" (Valid SHTP: len=%d, ch=%d)", len, data[2]);
            }
        }
        Serial.println();
    }
    
    // Restaurer 400kHz
    i2c_driver_delete(I2C_NUM_1);
    delay(100);
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)50;
    conf.scl_io_num = (gpio_num_t)49;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    i2c_param_config(I2C_NUM_1, &conf);
    i2c_driver_install(I2C_NUM_1, conf.mode, 0, 0, 0);
    i2c_set_timeout(I2C_NUM_1, 16000000);
}

void test_bno080_shtp() {
    Serial.println("\n2. Test protocole SHTP:");
    
    // Envoyer différentes commandes SHTP pour voir la réaction
    
    // A. Product ID Request
    Serial.println("\nA. Requesting Product ID:");
    uint8_t prodid_cmd[] = {
        0x06, 0x00,  // Length: 6 bytes
        0x02,        // Channel: CONTROL
        0x00,        // Seq
        0xF9, 0x00   // PRODUCT_ID_REQUEST
    };
    
    send_and_receive(prodid_cmd, 6);
    
    // B. Reset
    Serial.println("\nB. Sending Reset:");
    uint8_t reset_cmd[] = {
        0x05, 0x00,  // Length: 5
        0x01,        // Channel: EXECUTABLE
        0x00,        // Seq
        0x01         // Reset
    };
    
    send_and_receive(reset_cmd, 5);
    delay(2000);
    
    // C. Initialize
    Serial.println("\nC. Sending Initialize:");
    uint8_t init_cmd[] = {
        0x06, 0x00,  // Length: 6
        0x02,        // Channel: CONTROL
        0x00,        // Seq
        0x04, 0x00   // INITIALIZE
    };
    
    send_and_receive(init_cmd, 6);
    delay(500);
    
    // D. Get Feature Response
    Serial.println("\nD. Get Feature Response:");
    uint8_t getfeat_cmd[] = {
        0x06, 0x00,  // Length: 6
        0x02,        // Channel: CONTROL
        0x00,        // Seq
        0xFE, 0x05   // GET_FEATURE for rotation vector
    };
    
    send_and_receive(getfeat_cmd, 6);
}

void send_and_receive(uint8_t* cmd, int len) {
    // Envoyer
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (0x4A << 1) | I2C_MASTER_WRITE, false);
    for (int i = 0; i < len; i++) {
        i2c_master_write_byte(handle, cmd[i], false);
    }
    i2c_master_stop(handle);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, handle, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(handle);
    
    Serial.printf("  Send: %s\n", esp_err_to_name(ret));
    
    delay(100);
    
    // Recevoir réponse
    for (int attempt = 0; attempt < 5; attempt++) {
        uint8_t resp[32];
        
        handle = i2c_cmd_link_create();
        i2c_master_start(handle);
        i2c_master_write_byte(handle, (0x4A << 1) | I2C_MASTER_READ, false);
        i2c_master_read(handle, resp, 31, I2C_MASTER_ACK);
        i2c_master_read_byte(handle, &resp[31], I2C_MASTER_NACK);
        i2c_master_stop(handle);
        
        ret = i2c_master_cmd_begin(I2C_NUM_1, handle, pdMS_TO_TICKS(200));
        i2c_cmd_link_delete(handle);
        
        if (ret == ESP_OK) {
            // Vérifier si ce n'est pas que des FF
            bool validData = false;
            for (int i = 0; i < 4; i++) {
                if (resp[i] != 0xFF) {
                    validData = true;
                    break;
                }
            }
            
            if (validData) {
                uint16_t len = resp[0] | (resp[1] << 8);
                len &= 0x7FFF;
                
                Serial.printf("  Response %d: len=%d, ch=%d, seq=%d\n", 
                              attempt, len, resp[2], resp[3]);
                Serial.print("    Data: ");
                for (int i = 0; i < min(16, (int)len); i++) {
                    Serial.printf("%02X ", resp[i]);
                }
                Serial.println();
                
                if (len > 4 && resp[2] == 0x02) {  // Control channel
                    Serial.printf("    Report: 0x%02X 0x%02X\n", resp[4], resp[5]);
                }
            }
        }
        
        delay(100);
    }
}

void test_bno080_polling() {
    Serial.println("\n3. Polling intensif (10 secondes):");
    
    BNO08x_ESP32_P4 bno;
    if (!bno.begin(1, 0x4A)) {
        Serial.println("Failed to init BNO080");
        return;
    }
    
    // Essayer d'activer différents rapports
    Serial.println("Enabling reports...");
    bno.enableRotationVector(100);
    delay(500);
    bno.enableAccelerometer(100);
    delay(500);
    bno.enableGyro(100);
    delay(500);
    
    // Polling
    Serial.println("Starting polling...");
    uint32_t start = millis();
    int dataCount = 0;
    
    while (millis() - start < 10000) {
        if (bno.dataAvailable()) {
            dataCount++;
            Serial.printf("[%d] Quat: i=%.3f j=%.3f k=%.3f r=%.3f\n",
                          dataCount,
                          bno.getQuatI(), bno.getQuatJ(),
                          bno.getQuatK(), bno.getQuatReal());
        }
        
        // Essayer aussi une lecture directe
        uint8_t buffer[32];
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (0x4A << 1) | I2C_MASTER_READ, false);
        i2c_master_read(cmd, buffer, 31, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &buffer[31], I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        
        if (i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(50)) == ESP_OK) {
            // Vérifier si données valides
            if (buffer[0] != 0xFF || buffer[1] != 0xFF) {
                uint16_t len = buffer[0] | (buffer[1] << 8);
                len &= 0x7FFF;
                if (len > 0 && len < 128) {
                    Serial.printf("  Raw: len=%d ch=%d [%02X %02X %02X %02X...]\n",
                                  len, buffer[2], 
                                  buffer[4], buffer[5], buffer[6], buffer[7]);
                }
            }
        }
        i2c_cmd_link_delete(cmd);
        
        delay(50);
    }
    
    Serial.printf("\nTotal data packets: %d\n", dataCount);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("=================================");
  Serial.println("  Variometer ESP32-P4 Starting");
  Serial.println("=================================");

  // 1. Reset GT911 touch controller
  Serial.println("[INIT] Resetting touch controller...");
  pinMode(23, OUTPUT);
  digitalWrite(23, LOW);
  delay(10);
  digitalWrite(23, HIGH);
  delay(50);

  // 2. SD Manager
  Serial.println("[INIT] Initializing SD Manager...");
  if (!sd_manager_init()) {
    Serial.println("[INIT] SD Manager failed, continuing without SD");
  }

  // 3. Configuration
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

  // 4. Logger
  Serial.println("[INIT] Initializing logger...");
  if (!logger_init()) {
    Serial.println("[INIT] Logger initialization failed");
  }

  // 5. Memory monitor
  Serial.println("[INIT] Initializing memory monitor...");
  memory_monitor_init();

  // 6. Display (init I2C Bus 0 pour GT911)
  Serial.println("[INIT] Initializing Display...");
  if (!display_init_board()) {
    Serial.println("[FATAL] Board init failed!");
    while (1) delay(1000);
  }

  // 7. LVGL
  Serial.println("[INIT] Initializing LVGL...");
  if (!display_init_lvgl()) {
    Serial.println("[FATAL] LVGL init failed!");
    while (1) delay(1000);
  }

  // 8. I2C Bus 1 (capteurs)
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

  test_bno080_deep();

  // 9. Capteurs
  Serial.println("[INIT] Initializing sensors...");
  if (!sensor_init_all()) {
    Serial.println("[WARNING] Sensor init had errors");
  }
  sensor_init_print_summary();

  // 10. USB MSC
  Serial.println("[INIT] Initializing USB MSC...");
  if (!usb_msc_init()) {
    Serial.println("[WARNING] USB MSC init failed");
  }

  // 11. UI simple
  Serial.println("[INIT] Creating UI...");

  lv_obj_t* scr = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);

  // Titre
  lv_obj_t* label_title = lv_label_create(scr);
  lv_label_set_text(label_title, "FLIGHT TASK RUNNING");
  lv_obj_set_style_text_font(label_title, &lv_font_montserrat_32, 0);
  lv_obj_set_style_text_color(label_title, lv_color_hex(0x00FF00), 0);
  lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 20);

  // Label données de vol
  label_flight = lv_label_create(scr);
  lv_label_set_text(label_flight, "Waiting for data...");
  lv_obj_set_style_text_font(label_flight, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(label_flight, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(label_flight, LV_ALIGN_CENTER, 0, 0);

  lv_screen_load(scr);
  lv_obj_update_layout(scr);
  lv_refr_now(g_display);

  // 12. Task Flight
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

  // CRITIQUE : Handler LVGL (appelé régulièrement)
  display_task();

  // Test périodique capteurs (5s)
  if (millis() - last_print > 5000) {
    last_print = millis();

    LOG_I(LOG_MODULE_SYSTEM, "=== Status ===");

    // BMP5
    if (sensor_bmp5_ready) {
      bmp5_data_t bmp;
      if (BMP5_read(&bmp5_dev, &bmp, 1013.25f)) {
        LOG_I(LOG_MODULE_BMP5, "T=%.1f°C P=%.0fhPa Alt=%.0fm",
              bmp.temperature, bmp.pressure, bmp.altitude);
      }
    }

    // GPS
    if (sensor_gps_ready && gps_data.fix) {
      LOG_I(LOG_MODULE_GPS, "Fix=%d Sats=%d Alt=%.0fm",
            gps_data.fix, gps_data.satellites, gps_data.altitude);
    }

    // Battery
    if (sensor_battery_ready) {
      max17048_data_t bat;
      if (MAX17048_read(&max17048_dev, &bat)) {
        LOG_I(LOG_MODULE_SYSTEM, "Bat: %.0f%% (%.2fV)", bat.soc, bat.voltage);
      }
    }
  }

  // Petit délai pour éviter watchdog
  delay(5);
}