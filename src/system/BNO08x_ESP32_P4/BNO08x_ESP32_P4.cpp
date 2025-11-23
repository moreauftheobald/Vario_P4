/**
 * @file BNO08x_ESP32_P4_SparkFun.cpp
 * @brief Implémentation style SparkFun avec driver/i2c.h
 */

#include "BNO08x_ESP32_P4.h"
#include "src/system/logger/logger.h"

extern bool i2c_lock(i2c_bus_id_t bus, uint32_t timeout_ms);
extern void i2c_unlock(i2c_bus_id_t bus);

BNO08x_ESP32_P4::BNO08x_ESP32_P4() 
: _i2c_bus(0), _i2c_addr(0x4A), _i2c_port(I2C_NUM_0), packetLength(0) {
    quatI = quatJ = quatK = quatReal = quatRadianAccuracy = 0;
}

bool BNO08x_ESP32_P4::begin(uint8_t i2c_bus, uint8_t address) {
    _i2c_bus = i2c_bus;
    _i2c_addr = address;
    _i2c_port = (i2c_bus == 1) ? I2C_NUM_1 : I2C_NUM_0;
    
    LOG_I(LOG_MODULE_IMU, "BNO08x SparkFun: Starting on bus %d addr 0x%02X", i2c_bus, address);
    
    // Attendre stabilisation
    delay(100);
    
    // Soft reset (optionnel, peut échouer sur certains modules)
    uint8_t cmd = 0x01; // Reset command
    if (!sendPacket(CHANNEL_EXECUTABLE, 1)) {
        LOG_W(LOG_MODULE_IMU, "Soft reset failed (continuing anyway)");
    }
    
    delay(300);
    
    // Attendre advertisement
    uint32_t start = millis();
    while (millis() - start < 2000) {
        if (receivePacket()) {
            LOG_I(LOG_MODULE_IMU, "Advertisement received");
            return true;
        }
        delay(100);
    }
    
    LOG_W(LOG_MODULE_IMU, "No advertisement received");
    return true; // Continuer quand même
}

bool BNO08x_ESP32_P4::dataAvailable() {
    if (receivePacket()) {
        if (shtpHeader[2] == CHANNEL_REPORTS || shtpHeader[2] == CHANNEL_WAKE_REPORTS) {
            parseInputReport();
            return true;
        }
    }
    return false;
}

bool BNO08x_ESP32_P4::receivePacket() {
    i2c_bus_id_t bus_id = (_i2c_bus == 1) ? I2C_BUS_1 : I2C_BUS_0;
    
    if (!i2c_lock(bus_id, 100)) {
        return false;
    }
    
    // Lire header (4 bytes)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, shtpHeader, 4, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (err != ESP_OK) {
        i2c_unlock(bus_id);
        return false;
    }
    
    // Parser taille
    packetLength = (uint16_t)shtpHeader[0] | ((uint16_t)shtpHeader[1] << 8);
    packetLength &= 0x7FFF; // Clear continuation bit
    
    if (packetLength == 0 || packetLength == 0xFFFF) {
        i2c_unlock(bus_id);
        return false;
    }
    
    if (packetLength > MAX_PACKET_SIZE) {
        i2c_unlock(bus_id);
        return false;
    }
    
    // Lire data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, shtpData, packetLength, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    err = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    i2c_unlock(bus_id);
    
    return (err == ESP_OK);
}

bool BNO08x_ESP32_P4::sendPacket(uint8_t channelNumber, uint8_t dataLength) {
    i2c_bus_id_t bus_id = (_i2c_bus == 1) ? I2C_BUS_1 : I2C_BUS_0;
    
    uint16_t totalLength = dataLength + 4;
    
    if (!i2c_lock(bus_id, 100)) {
        return false;
    }
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2c_addr << 1) | I2C_MASTER_WRITE, true);
    
    // Header
    i2c_master_write_byte(cmd, totalLength & 0xFF, true);
    i2c_master_write_byte(cmd, totalLength >> 8, true);
    i2c_master_write_byte(cmd, channelNumber, true);
    i2c_master_write_byte(cmd, 0, true); // Sequence number
    
    // Data
    for (int i = 0; i < dataLength; i++) {
        i2c_master_write_byte(cmd, shtpData[i], true);
    }
    
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    i2c_unlock(bus_id);
    
    return (err == ESP_OK);
}

void BNO08x_ESP32_P4::parseInputReport() {
    if (packetLength < 5) return;
    
    uint8_t reportID = shtpData[0];
    
    if (reportID == 0x05) { // Rotation vector
        quatI = ((int16_t)((shtpData[10] << 8) | shtpData[9])) / 16384.0f;
        quatJ = ((int16_t)((shtpData[12] << 8) | shtpData[11])) / 16384.0f;
        quatK = ((int16_t)((shtpData[14] << 8) | shtpData[13])) / 16384.0f;
        quatReal = ((int16_t)((shtpData[16] << 8) | shtpData[15])) / 16384.0f;
        quatRadianAccuracy = ((int16_t)((shtpData[18] << 8) | shtpData[17])) / 16384.0f;
    }
}

bool BNO08x_ESP32_P4::enableRotationVector(uint16_t timeBetweenReports) {
    setFeatureCommand(0x05, timeBetweenReports);
    return sendPacket(CHANNEL_CONTROL, 17);
}

void BNO08x_ESP32_P4::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports) {
    uint32_t microsBetweenReports = (uint32_t)timeBetweenReports * 1000;
    
    shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    shtpData[1] = reportID;
    shtpData[2] = 0; // Feature flags
    shtpData[3] = 0;
    shtpData[4] = (microsBetweenReports >> 0) & 0xFF;
    shtpData[5] = (microsBetweenReports >> 8) & 0xFF;
    shtpData[6] = (microsBetweenReports >> 16) & 0xFF;
    shtpData[7] = (microsBetweenReports >> 24) & 0xFF;
    shtpData[8] = 0;  // Batch interval
    shtpData[9] = 0;
    shtpData[10] = 0;
    shtpData[11] = 0;
    shtpData[12] = 0; // Sensor specific
    shtpData[13] = 0;
    shtpData[14] = 0;
    shtpData[15] = 0;
    shtpData[16] = 0;
}

float BNO08x_ESP32_P4::getQuatI() { return quatI; }
float BNO08x_ESP32_P4::getQuatJ() { return quatJ; }
float BNO08x_ESP32_P4::getQuatK() { return quatK; }
float BNO08x_ESP32_P4::getQuatReal() { return quatReal; }
float BNO08x_ESP32_P4::getQuatRadianAccuracy() { return quatRadianAccuracy; }