/**
 * @file BNO08x_ESP32_P4.cpp
 * @brief Driver BNO080 pour ESP32-P4 avec ancienne API I2C
 */

#include "BNO08x_ESP32_P4.h"
#include "esp_log.h"

#define LOG_TAG "BNO080"

BNO08x_ESP32_P4* BNO08x_ESP32_P4::_instance = nullptr;
static sh2_SensorValue_t *_sensor_value = nullptr;
static bool _reset_occurred = false;

// ============================================================================
// CALLBACKS SH2
// ============================================================================

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
    if (pEvent->eventId == SH2_RESET) {
        _reset_occurred = true;
    }
}

static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent) {
    if (_sensor_value) {
        int rc = sh2_decodeSensorEvent(_sensor_value, pEvent);
        if (rc != SH2_OK) {
            _sensor_value->timestamp = 0;
        }
    }
}

// ============================================================================
// CONSTRUCTEUR / DESTRUCTEUR
// ============================================================================

BNO08x_ESP32_P4::BNO08x_ESP32_P4(int8_t reset_pin)
: _reset_pin(reset_pin), _i2c_port(I2C_NUM_0), _i2c_addr(BNO08X_I2CADDR_DEFAULT) {
    _instance = this;
}

BNO08x_ESP32_P4::~BNO08x_ESP32_P4() {
    _instance = nullptr;
}

// ============================================================================
// HAL CALLBACKS (accès I2C avec ancienne API)
// ============================================================================

int BNO08x_ESP32_P4::hal_open(sh2_Hal_t *self) {
    if (!_instance) return -1;

    // Soft reset du BNO080 via I2C
    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    bool success = false;

    for (uint8_t attempts = 0; attempts < 5; attempts++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_instance->_i2c_addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, softreset_pkt, sizeof(softreset_pkt), true);
        i2c_master_stop(cmd);
        
        esp_err_t err = i2c_master_cmd_begin(_instance->_i2c_port, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        
        if (err == ESP_OK) {
            success = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    if (!success) {
        ESP_LOGE(LOG_TAG, "Failed to send soft reset");
        return -1;
    }

    vTaskDelay(pdMS_TO_TICKS(300));  // Attendre stabilisation
    return 0;
}

void BNO08x_ESP32_P4::hal_close(sh2_Hal_t *self) {
    // Rien à faire
}

int BNO08x_ESP32_P4::hal_read(sh2_Hal_t *self, uint8_t *pBuffer,
                               unsigned len, uint32_t *t_us) {
    if (!_instance) return 0;

    // Lire header SHTP (4 bytes)
    uint8_t header[4];
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_instance->_i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, header, 4, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(_instance->_i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (err != ESP_OK) return 0;

    // Taille paquet (bits 0-14, bit 15 = continuation)
    uint16_t packet_size = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
    packet_size &= ~0x8000;

    if (packet_size > len || packet_size < 4) return 0;

    // Lecture du paquet complet
    if (packet_size <= 128) {
        // Petit paquet : lecture directe
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_instance->_i2c_addr << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, pBuffer, packet_size, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        
        err = i2c_master_cmd_begin(_instance->_i2c_port, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
        
        if (err != ESP_OK) return 0;
    } else {
        // Grand paquet : lecture par blocs (max 128 bytes I2C)
        memcpy(pBuffer, header, 4);
        uint16_t remaining = packet_size - 4;
        uint16_t cursor = 4;

        while (remaining > 0) {
            uint16_t chunk_size = (remaining > 124) ? 124 : remaining;
            uint8_t temp[128];
            
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (_instance->_i2c_addr << 1) | I2C_MASTER_READ, true);
            i2c_master_read(cmd, temp, chunk_size + 4, I2C_MASTER_LAST_NACK);
            i2c_master_stop(cmd);
            
            err = i2c_master_cmd_begin(_instance->_i2c_port, cmd, pdMS_TO_TICKS(1000));
            i2c_cmd_link_delete(cmd);
            
            if (err != ESP_OK) return 0;

            memcpy(pBuffer + cursor, temp + 4, chunk_size);
            cursor += chunk_size;
            remaining -= chunk_size;
        }
    }

    *t_us = (uint32_t)(esp_timer_get_time());
    return packet_size;
}

int BNO08x_ESP32_P4::hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    if (!_instance) return 0;

    // I2C buffer max = 128 bytes
    const size_t i2c_buffer_max = 128;
    uint16_t write_size = (len > i2c_buffer_max) ? i2c_buffer_max : len;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_instance->_i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, pBuffer, write_size, true);
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(_instance->_i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return (err == ESP_OK) ? write_size : 0;
}

uint32_t BNO08x_ESP32_P4::hal_getTimeUs(sh2_Hal_t *self) {
    return (uint32_t)(esp_timer_get_time());
}

// ============================================================================
// INITIALISATION
// ============================================================================

bool BNO08x_ESP32_P4::begin_I2C(i2c_port_t i2c_port,
                                 uint8_t i2c_addr,
                                 int32_t sensor_id) {
    _i2c_port = i2c_port;
    _i2c_addr = i2c_addr;

    ESP_LOGI(LOG_TAG, "Initializing BNO080 on I2C port %d, addr 0x%02X", 
             i2c_port, i2c_addr);

    // Configuration HAL
    _HAL.open = hal_open;
    _HAL.close = hal_close;
    _HAL.read = hal_read;
    _HAL.write = hal_write;
    _HAL.getTimeUs = hal_getTimeUs;

    return _init(sensor_id);
}

bool BNO08x_ESP32_P4::_init(int32_t sensor_id) {
    // Reset hardware si pin définie
    if (_reset_pin != -1) {
        pinMode(_reset_pin, OUTPUT);
        digitalWrite(_reset_pin, HIGH);
        delay(10);
        digitalWrite(_reset_pin, LOW);
        delay(10);
        digitalWrite(_reset_pin, HIGH);
        delay(50);
        ESP_LOGI(LOG_TAG, "Hardware reset completed");
    }

    // Ouvrir session SH2
    int status = sh2_open(&_HAL, hal_callback, nullptr);
    if (status != SH2_OK) {
        ESP_LOGE(LOG_TAG, "sh2_open failed: %d", status);
        return false;
    }

    // Attendre reset complete
    uint32_t start = millis();
    while (!_reset_occurred && (millis() - start) < 2000) {
        sh2_service();
        delay(10);
    }

    if (!_reset_occurred) {
        ESP_LOGW(LOG_TAG, "No reset event received (timeout)");
    }

    // Obtenir Product IDs
    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    if (status != SH2_OK) {
        ESP_LOGE(LOG_TAG, "sh2_getProdIds failed: %d", status);
        return false;
    }

    ESP_LOGI(LOG_TAG, "BNO080 found: %d product ID entries", prodIds.numEntries);

    // Enregistrer callback capteur
    sh2_setSensorCallback(sensorHandler, nullptr);

    return true;
}

// ============================================================================
// FONCTIONS PUBLIQUES
// ============================================================================

void BNO08x_ESP32_P4::hardwareReset() {
    if (_reset_pin != -1) {
        digitalWrite(_reset_pin, LOW);
        delay(10);
        digitalWrite(_reset_pin, HIGH);
        delay(50);
        ESP_LOGI(LOG_TAG, "Hardware reset triggered");
    }
}

bool BNO08x_ESP32_P4::wasReset() {
    bool x = _reset_occurred;
    _reset_occurred = false;
    return x;
}

bool BNO08x_ESP32_P4::getSensorEvent(sh2_SensorValue_t *value) {
    _sensor_value = value;
    value->timestamp = 0;

    sh2_service();

    if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
        return false;
    }

    return true;
}

bool BNO08x_ESP32_P4::enableReport(sh2_SensorId_t sensorId, uint32_t interval_us) {
    sh2_SensorConfig_t config;

    memset(&config, 0, sizeof(config));
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;
    config.reportInterval_us = interval_us;

    int status = sh2_setSensorConfig(sensorId, &config);
    
    if (status == SH2_OK) {
        ESP_LOGI(LOG_TAG, "Enabled sensor 0x%02X @ %d us", sensorId, interval_us);
    } else {
        ESP_LOGE(LOG_TAG, "Failed to enable sensor 0x%02X: %d", sensorId, status);
    }
    
    return (status == SH2_OK);
}