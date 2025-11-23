/**
 * @file BNO08x_ESP32_P4.cpp
 * @brief Driver BNO080 optimisé pour basse vitesse I2C
 */

#include "BNO08x_ESP32_P4.h"
#include "src/system/logger/logger.h"

extern bool i2c_lock(i2c_bus_id_t bus, uint32_t timeout_ms);
extern void i2c_unlock(i2c_bus_id_t bus);

// Variable globale pour la vitesse I2C du BNO080
static uint32_t bno080_i2c_speed = 50000;  // 50kHz optimal

BNO08x_ESP32_P4::BNO08x_ESP32_P4() 
: _i2c_bus(0), _i2c_addr(0x4A), _i2c_port(I2C_NUM_0), packetLength(0) {
    quatI = quatJ = quatK = quatReal = quatRadianAccuracy = 0;
}

// Fonction pour changer temporairement la vitesse I2C
static void set_i2c_speed(i2c_port_t port, uint32_t speed) {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)50;  // Vos pins
    conf.scl_io_num = (gpio_num_t)49;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = speed;
    conf.clk_flags = 0;
    
    i2c_param_config(port, &conf);
}

bool BNO08x_ESP32_P4::begin(uint8_t i2c_bus, uint8_t address) {
    _i2c_bus = i2c_bus;
    _i2c_addr = address;
    _i2c_port = (i2c_bus == 1) ? I2C_NUM_1 : I2C_NUM_0;
    
    LOG_I(LOG_MODULE_IMU, "BNO08x: Init on bus %d addr 0x%02X", i2c_bus, address);
    
    // Passer en basse vitesse pour le BNO080
    i2c_bus_id_t bus_id = (_i2c_bus == 1) ? I2C_BUS_1 : I2C_BUS_0;
    
    LOG_I(LOG_MODULE_IMU, "Setting I2C to 50kHz for BNO080");
    set_i2c_speed(_i2c_port, 50000);
    delay(100);
    
    // Test de présence
    LOG_I(LOG_MODULE_IMU, "Testing presence...");
    if (!waitForI2C()) {
        LOG_E(LOG_MODULE_IMU, "BNO080 not responding");
        // Restaurer vitesse normale
        set_i2c_speed(_i2c_port, 400000);
        return false;
    }
    
    LOG_I(LOG_MODULE_IMU, "BNO080 detected");
    
    // Attendre stabilisation
    delay(500);
    
    // Lire et traiter tous les messages d'initialisation
    LOG_I(LOG_MODULE_IMU, "Reading initialization messages...");
    int msgCount = 0;
    uint32_t start = millis();
    
    while (millis() - start < 2000) {
        if (receivePacket()) {
            msgCount++;
            LOG_I(LOG_MODULE_IMU, "Init msg %d: ch=%d, len=%d", 
                  msgCount, shtpHeader[2], packetLength);
            
            // Si c'est un message sur channel 0, l'afficher
            if (shtpHeader[2] == CHANNEL_COMMAND && packetLength > 4) {
                // Chercher du texte ASCII (version, etc.)
                for (int i = 0; i < packetLength - 4 && i < 32; i++) {
                    if (shtpData[i] >= 0x20 && shtpData[i] <= 0x7E) {
                        Serial.printf("%c", shtpData[i]);
                    }
                }
                Serial.println();
            }
            
            // Si on reçoit un message EXECUTABLE, c'est bon signe
            if (shtpHeader[2] == CHANNEL_EXECUTABLE) {
                LOG_I(LOG_MODULE_IMU, "Got EXECUTABLE response - sensor ready");
                break;
            }
        }
        delay(50);
    }
    
    LOG_I(LOG_MODULE_IMU, "Received %d init messages", msgCount);
    
    // Envoyer INITIALIZE si nécessaire
    if (msgCount == 0) {
        LOG_I(LOG_MODULE_IMU, "No init messages, sending INITIALIZE");
        memset(shtpData, 0, sizeof(shtpData));
        shtpData[0] = 0x04;  // INITIALIZE
        shtpData[1] = 0x00;  // Subcommand
        
        if (sendPacket(CHANNEL_CONTROL, 2)) {
            delay(500);
            
            // Lire réponses
            for (int i = 0; i < 10; i++) {
                if (receivePacket()) {
                    LOG_I(LOG_MODULE_IMU, "Init response: ch=%d", shtpHeader[2]);
                }
                delay(50);
            }
        }
    }
    
    // Essayer un soft reset pour nettoyer l'état
    LOG_I(LOG_MODULE_IMU, "Sending soft reset");
    memset(shtpData, 0, sizeof(shtpData));
    shtpData[0] = 0x01;  // Reset
    
    if (sendPacket(CHANNEL_EXECUTABLE, 1)) {
        LOG_I(LOG_MODULE_IMU, "Reset sent, waiting 3s");
        delay(3000);
        
        // Vider le buffer après reset
        /*while (receivePacket()) {
            LOG_I(LOG_MODULE_IMU, "Post-reset: ch=%d", shtpHeader[2]);
            delay(10);
        }*/
    }
    
    LOG_I(LOG_MODULE_IMU, "BNO080 initialized");
    
    // Restaurer vitesse normale pour les autres capteurs
    LOG_I(LOG_MODULE_IMU, "Restoring I2C to 400kHz");
    set_i2c_speed(_i2c_port, 400000);
    
    return true;
}

bool BNO08x_ESP32_P4::waitForI2C() {
    i2c_bus_id_t bus_id = (_i2c_bus == 1) ? I2C_BUS_1 : I2C_BUS_0;
    
    if (!i2c_lock(bus_id, 100)) {
        return false;
    }
    
    // Passer temporairement en basse vitesse
    set_i2c_speed(_i2c_port, bno080_i2c_speed);
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2c_addr << 1) | I2C_MASTER_WRITE, false);
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    // Restaurer vitesse normale
    set_i2c_speed(_i2c_port, 400000);
    
    i2c_unlock(bus_id);
    
    return (err == ESP_OK || err == ESP_ERR_TIMEOUT);
}

bool BNO08x_ESP32_P4::dataAvailable() {
    if (receivePacket()) {
        // Afficher ce qu'on reçoit pour debug
        LOG_I(LOG_MODULE_IMU, "Data: ch=%d, len=%d, report=0x%02X",
              shtpHeader[2], packetLength, 
              (packetLength > 4) ? shtpData[0] : 0);
        
        if (shtpHeader[2] == CHANNEL_REPORTS || 
            shtpHeader[2] == CHANNEL_WAKE_REPORTS) {
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
    
    // Passer en basse vitesse pour le BNO080
    set_i2c_speed(_i2c_port, bno080_i2c_speed);
    
    // Lire le header
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2c_addr << 1) | I2C_MASTER_READ, false);
    
    for (int i = 0; i < 3; i++) {
        i2c_master_read_byte(cmd, &shtpHeader[i], I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &shtpHeader[3], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    
    if (err != ESP_OK) {
        set_i2c_speed(_i2c_port, 400000);
        i2c_unlock(bus_id);
        return false;
    }
    
    // Parser la taille
    packetLength = shtpHeader[0] | ((uint16_t)(shtpHeader[1] & 0x7F) << 8);
    
    // Vérifications
    if (packetLength == 0 || packetLength > MAX_PACKET_SIZE) {
        set_i2c_speed(_i2c_port, 400000);
        i2c_unlock(bus_id);
        return false;
    }
    
    // Lire les données si nécessaire
    uint16_t dataLength = packetLength - 4;
    if (dataLength > 0) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_i2c_addr << 1) | I2C_MASTER_READ, false);
        
        if (dataLength > 1) {
            i2c_master_read(cmd, shtpData, dataLength - 1, I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, &shtpData[dataLength - 1], I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        
        err = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(200));
        i2c_cmd_link_delete(cmd);
    }
    
    // Restaurer vitesse normale
    set_i2c_speed(_i2c_port, 400000);
    i2c_unlock(bus_id);
    
    return (err == ESP_OK);
}

bool BNO08x_ESP32_P4::sendPacket(uint8_t channelNumber, uint8_t dataLength) {
    i2c_bus_id_t bus_id = (_i2c_bus == 1) ? I2C_BUS_1 : I2C_BUS_0;
    
    uint16_t totalLength = dataLength + 4;
    
    uint8_t txPacket[MAX_PACKET_SIZE];
    txPacket[0] = totalLength & 0xFF;
    txPacket[1] = (totalLength >> 8) & 0x7F;
    txPacket[2] = channelNumber;
    txPacket[3] = 0;  // Seq
    
    memcpy(&txPacket[4], shtpData, dataLength);
    
    if (!i2c_lock(bus_id, 100)) {
        return false;
    }
    
    // Basse vitesse pour envoi
    set_i2c_speed(_i2c_port, bno080_i2c_speed);
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2c_addr << 1) | I2C_MASTER_WRITE, false);
    
    for (int i = 0; i < totalLength; i++) {
        i2c_master_write_byte(cmd, txPacket[i], false);
    }
    
    i2c_master_stop(cmd);
    
    esp_err_t err = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(200));
    i2c_cmd_link_delete(cmd);
    
    // Restaurer vitesse normale
    set_i2c_speed(_i2c_port, 400000);
    i2c_unlock(bus_id);
    
    if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
        LOG_W(LOG_MODULE_IMU, "Send failed: %s", esp_err_to_name(err));
        return false;
    }
    
    delay(50);  // Plus de temps après envoi
    return true;
}

void BNO08x_ESP32_P4::parseInputReport() {
    if (packetLength < 5) return;
    
    uint8_t reportID = shtpData[0];
    
    if (reportID == 0x05 && packetLength >= 19) {
        // Rotation vector - les données sont au bon endroit
        int16_t rawI = (int16_t)(shtpData[10] << 8 | shtpData[9]);
        int16_t rawJ = (int16_t)(shtpData[12] << 8 | shtpData[11]);
        int16_t rawK = (int16_t)(shtpData[14] << 8 | shtpData[13]);
        int16_t rawReal = (int16_t)(shtpData[16] << 8 | shtpData[15]);
        int16_t rawAcc = (int16_t)(shtpData[18] << 8 | shtpData[17]);
        
        quatI = rawI / 16384.0f;
        quatJ = rawJ / 16384.0f;
        quatK = rawK / 16384.0f;
        quatReal = rawReal / 16384.0f;
        quatRadianAccuracy = rawAcc / 16384.0f;
        
        LOG_I(LOG_MODULE_IMU, "Quaternion updated: i=%.3f j=%.3f k=%.3f r=%.3f",
              quatI, quatJ, quatK, quatReal);
    }
}

bool BNO08x_ESP32_P4::enableRotationVector(uint16_t timeBetweenReports) {
    LOG_I(LOG_MODULE_IMU, "Enabling rotation vector @ %dms", timeBetweenReports);
    
    // Utiliser basse vitesse pour configuration
    i2c_bus_id_t bus_id = (_i2c_bus == 1) ? I2C_BUS_1 : I2C_BUS_0;
    
    // Préparer la commande
    setFeatureCommand(0x05, timeBetweenReports);
    
    // Envoyer plusieurs fois pour être sûr
    bool success = false;
    for (int retry = 0; retry < 3; retry++) {
        if (sendPacket(CHANNEL_CONTROL, 17)) {
            delay(500);
            
            // Chercher une confirmation
            for (int i = 0; i < 10; i++) {
                if (receivePacket()) {
                    if (shtpHeader[2] == CHANNEL_CONTROL) {
                        LOG_I(LOG_MODULE_IMU, "Feature command acknowledged");
                        success = true;
                        break;
                    }
                }
                delay(50);
            }
            
            if (success) break;
        }
        
        LOG_W(LOG_MODULE_IMU, "Retry %d enabling feature", retry + 1);
        delay(500);
    }
    
    return success;
}

void BNO08x_ESP32_P4::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports) {
    uint32_t microsBetweenReports = (uint32_t)timeBetweenReports * 1000L;
    
    memset(shtpData, 0, 17);
    
    shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    shtpData[1] = reportID;
    shtpData[2] = 0;
    shtpData[3] = 0;
    shtpData[4] = 0;
    shtpData[5] = (microsBetweenReports >> 0) & 0xFF;
    shtpData[6] = (microsBetweenReports >> 8) & 0xFF;
    shtpData[7] = (microsBetweenReports >> 16) & 0xFF;
    shtpData[8] = (microsBetweenReports >> 24) & 0xFF;
    shtpData[9] = 0;
    shtpData[10] = 0;
    shtpData[11] = 0;
    shtpData[12] = 0;
    shtpData[13] = 0;
    shtpData[14] = 0;
    shtpData[15] = 0;
    shtpData[16] = 0;
}

// Getters
float BNO08x_ESP32_P4::getQuatI() { return quatI; }
float BNO08x_ESP32_P4::getQuatJ() { return quatJ; }
float BNO08x_ESP32_P4::getQuatK() { return quatK; }
float BNO08x_ESP32_P4::getQuatReal() { return quatReal; }
float BNO08x_ESP32_P4::getQuatRadianAccuracy() { return quatRadianAccuracy; }

// Autres capteurs
bool BNO08x_ESP32_P4::enableAccelerometer(uint16_t timeBetweenReports) {
    setFeatureCommand(0x01, timeBetweenReports);
    return sendPacket(CHANNEL_CONTROL, 17);
}

bool BNO08x_ESP32_P4::enableGyro(uint16_t timeBetweenReports) {
    setFeatureCommand(0x02, timeBetweenReports);
    return sendPacket(CHANNEL_CONTROL, 17);
}

bool BNO08x_ESP32_P4::enableLinearAccelerometer(uint16_t timeBetweenReports) {
    setFeatureCommand(0x04, timeBetweenReports);
    return sendPacket(CHANNEL_CONTROL, 17);
}