#include "BNO08x_ESP32.h"

BNO08x_ESP32* BNO08x_ESP32::_instance = nullptr;
static sh2_SensorValue_t *_sensor_value = nullptr;
static bool _reset_occurred = false;

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

BNO08x_ESP32::BNO08x_ESP32(int8_t reset_pin)
: _reset_pin(reset_pin), _bus_handle(nullptr), _dev_handle(nullptr), _owns_bus(false) {
    _instance = this;
}

BNO08x_ESP32::~BNO08x_ESP32() {
    if (_dev_handle) {
        i2c_master_bus_rm_device(_dev_handle);
        _dev_handle = nullptr;
    }
    _instance = nullptr;
}

// Reproduction exacte du comportement Wire
int BNO08x_ESP32::hal_open(sh2_Hal_t *self) {
    if (!_instance || !_instance->_dev_handle) return -1;

    uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
    bool success = false;

    for (uint8_t attempts = 0; attempts < 5; attempts++) {
        esp_err_t err = i2c_master_transmit(_instance->_dev_handle,
                                            softreset_pkt,
                                            sizeof(softreset_pkt),
                                            1000);
        if (err == ESP_OK) {
            success = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    if (!success) return -1;

    vTaskDelay(pdMS_TO_TICKS(300));
    return 0;
}

void BNO08x_ESP32::hal_close(sh2_Hal_t *self) {
    // Rien à faire
}

int BNO08x_ESP32::hal_read(sh2_Hal_t *self, uint8_t *pBuffer,
                           unsigned len, uint32_t *t_us) {
    if (!_instance || !_instance->_dev_handle) return 0;

    // Lire header (4 bytes)
    uint8_t header[4];
    esp_err_t err = i2c_master_receive(_instance->_dev_handle, header, 4, 1000);
    if (err != ESP_OK) return 0;

    // Déterminer taille paquet
    uint16_t packet_size = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
    packet_size &= ~0x8000;

    if (packet_size > len || packet_size < 4) return 0;

    // Si petit paquet (< 128 bytes), lecture directe
    if (packet_size <= 128) {
        err = i2c_master_receive(_instance->_dev_handle, pBuffer, packet_size, 1000);
        if (err != ESP_OK) return 0;

        *t_us = (uint32_t)(esp_timer_get_time());
        return packet_size;
    }

    // Sinon, lecture par blocs
    memcpy(pBuffer, header, 4);
    uint16_t remaining = packet_size - 4;
    uint16_t cursor = 4;

    while (remaining > 0) {
        uint16_t chunk_size = (remaining > 124) ? 124 : remaining;  // 128 - 4 bytes header

        uint8_t temp[128];
        err = i2c_master_receive(_instance->_dev_handle, temp, chunk_size + 4, 1000);
        if (err != ESP_OK) return 0;

        memcpy(pBuffer + cursor, temp + 4, chunk_size);
        cursor += chunk_size;
        remaining -= chunk_size;
    }

    *t_us = (uint32_t)(esp_timer_get_time());
    return packet_size;
                           }

                           int BNO08x_ESP32::hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
                               if (!_instance || !_instance->_dev_handle) return 0;

                               const size_t i2c_buffer_max = 128;
                               uint16_t write_size = (len > i2c_buffer_max) ? i2c_buffer_max : len;

                               esp_err_t err = i2c_master_transmit(_instance->_dev_handle,
                                                                   pBuffer,
                                                                   write_size,
                                                                   1000);

                               return (err == ESP_OK) ? write_size : 0;
                           }

                           uint32_t BNO08x_ESP32::hal_getTimeUs(sh2_Hal_t *self) {
                               return (uint32_t)(esp_timer_get_time());
                           }

                           bool BNO08x_ESP32::begin_I2C(i2c_master_bus_handle_t bus_handle,
                                                        uint8_t i2c_addr,
                                                        uint32_t freq,
                                                        int32_t sensor_id) {
                               if (bus_handle == nullptr) {
                                   return false;
                               }

                               _i2c_addr = i2c_addr;
                               _bus_handle = bus_handle;
                               _owns_bus = false;

                               // Configuration device
                               i2c_device_config_t dev_config = {};
                               dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
                               dev_config.device_address = i2c_addr;
                               dev_config.scl_speed_hz = freq;

                               esp_err_t err = i2c_master_bus_add_device(_bus_handle, &dev_config, &_dev_handle);
                               if (err != ESP_OK) {
                                   return false;
                               }

                               // Configuration HAL
                               _HAL.open = hal_open;
                               _HAL.close = hal_close;
                               _HAL.read = hal_read;
                               _HAL.write = hal_write;
                               _HAL.getTimeUs = hal_getTimeUs;

                               return _init(sensor_id);
                                                        }

                                                        bool BNO08x_ESP32::_init(int32_t sensor_id) {
                                                            // Reset hardware si pin définie
                                                            if (_reset_pin != -1) {
                                                                pinMode(_reset_pin, OUTPUT);
                                                                digitalWrite(_reset_pin, HIGH);
                                                                delay(10);
                                                                digitalWrite(_reset_pin, LOW);
                                                                delay(10);
                                                                digitalWrite(_reset_pin, HIGH);
                                                                delay(10);
                                                            }

                                                            // Ouvrir session SH2
                                                            int status = sh2_open(&_HAL, hal_callback, nullptr);
                                                            if (status != SH2_OK) {
                                                                return false;
                                                            }

                                                            // Obtenir Product IDs
                                                            memset(&prodIds, 0, sizeof(prodIds));
                                                            status = sh2_getProdIds(&prodIds);
                                                            if (status != SH2_OK) {
                                                                return false;
                                                            }

                                                            // Enregistrer callback pour événements capteur
                                                            sh2_setSensorCallback(sensorHandler, nullptr);

                                                            return true;
                                                        }

                                                        void BNO08x_ESP32::hardwareReset() {
                                                            if (_reset_pin != -1) {
                                                                digitalWrite(_reset_pin, LOW);
                                                                delay(10);
                                                                digitalWrite(_reset_pin, HIGH);
                                                                delay(10);
                                                            }
                                                        }

                                                        bool BNO08x_ESP32::wasReset() {
                                                            bool x = _reset_occurred;
                                                            _reset_occurred = false;
                                                            return x;
                                                        }

                                                        bool BNO08x_ESP32::getSensorEvent(sh2_SensorValue_t *value) {
                                                            _sensor_value = value;
                                                            value->timestamp = 0;

                                                            sh2_service();

                                                            if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
                                                                return false;
                                                            }

                                                            return true;
                                                        }

                                                        bool BNO08x_ESP32::enableReport(sh2_SensorId_t sensorId, uint32_t interval_us) {
                                                            sh2_SensorConfig_t config;

                                                            config.changeSensitivityEnabled = false;
                                                            config.wakeupEnabled = false;
                                                            config.changeSensitivityRelative = false;
                                                            config.alwaysOnEnabled = false;
                                                            config.changeSensitivity = 0;
                                                            config.batchInterval_us = 0;
                                                            config.sensorSpecific = 0;
                                                            config.reportInterval_us = interval_us;

                                                            int status = sh2_setSensorConfig(sensorId, &config);
                                                            return (status == SH2_OK);
                                                        }
