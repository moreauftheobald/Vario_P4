/**
 * @file BNO08x_ESP32_P4_SparkFun.h
 * @brief Driver BNO080 style SparkFun adapté pour driver/i2c.h
 * 
 * Basé sur SparkFun_BNO080_Arduino_Library mais sans Wire
 */

#ifndef BNO08X_ESP32_P4
#define BNO08X_ESP32_P4

#include <Arduino.h>
#include <driver/i2c.h>
#include "src/hal/i2c_wrapper/i2c_wrapper.h"

// Commandes SHTP
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD  // ✅ AJOUTER

// Canaux SHTP
#define CHANNEL_COMMAND 0
#define CHANNEL_EXECUTABLE 1
#define CHANNEL_CONTROL 2
#define CHANNEL_REPORTS 3
#define CHANNEL_WAKE_REPORTS 4
#define CHANNEL_GYRO 5

// Tailles
#define MAX_PACKET_SIZE 128
#define MAX_METADATA_SIZE 9

class BNO08x_ESP32_P4 {
public:
    BNO08x_ESP32_P4();
    
    bool begin(uint8_t i2c_bus, uint8_t address = 0x4A);
    
    bool dataAvailable();
    void parseInputReport();
    
    float getQuatI();
    float getQuatJ();
    float getQuatK();
    float getQuatReal();
    float getQuatRadianAccuracy();
    
    bool enableRotationVector(uint16_t timeBetweenReports);
    bool enableAccelerometer(uint16_t timeBetweenReports);
    bool enableGyro(uint16_t timeBetweenReports);
    bool enableLinearAccelerometer(uint16_t timeBetweenReports);
    
private:
    uint8_t _i2c_bus;
    uint8_t _i2c_addr;
    i2c_port_t _i2c_port;
    
    // Buffers
    uint8_t shtpHeader[4];
    uint8_t shtpData[MAX_PACKET_SIZE];
    uint16_t packetLength;
    
    // Données quaternion
    float quatI, quatJ, quatK, quatReal, quatRadianAccuracy;
    
    // Fonctions I2C bas niveau
    bool receivePacket();
    bool sendPacket(uint8_t channelNumber, uint8_t dataLength);
    bool waitForI2C();
    
    // Helpers
    void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
    uint16_t parseCommandReport();
};

#endif