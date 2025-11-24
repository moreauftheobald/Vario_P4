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

bool BNO08x_ESP32_P4::begin(uint8_t i2c_bus, uint8_t address) {
  _i2c_bus = i2c_bus;
  _i2c_addr = address;
  _i2c_port = (i2c_bus == 1) ? I2C_NUM_1 : I2C_NUM_0;

  LOG_I(LOG_MODULE_IMU, "BNO08x: Init on bus %d addr 0x%02X", i2c_bus, address);

  if (!waitForI2C()) {
    LOG_E(LOG_MODULE_IMU, "BNO080 not responding");
    return false;
  }

  LOG_I(LOG_MODULE_IMU, "BNO080 detected");

  // Soft reset
  LOG_I(LOG_MODULE_IMU, "Sending soft reset...");
  shtpData[0] = 1;
  if (sendPacket(CHANNEL_EXECUTABLE, 1)) {
    delay(300);
    LOG_I(LOG_MODULE_IMU, "Reset sent, waiting for boot");
    delay(1500);  // ✅ Réduire à 500ms
  }

  // Remplacer la section "Reading init messages"
  LOG_I(LOG_MODULE_IMU, "Reading init messages...");
  int flushCount = 0;
  bool gotAdvertisement = false;

  // ✅ Max 20 tentatives au lieu de boucle infinie
  for (int attempt = 0; attempt < 20; attempt++) {
    if (receivePacket()) {
      if (packetLength > 4) {
        LOG_I(LOG_MODULE_IMU, "Init msg %d: ch=%d, report=0x%02X",
              flushCount, shtpHeader[2], shtpData[0]);

        if (shtpData[0] == 0x00 || shtpData[0] == 0xF8) {
          gotAdvertisement = true;
        }
      }
      flushCount++;
    }
    delay(100);  // ✅ Délai HORS du lock (receivePacket libère le lock)
  }

  LOG_I(LOG_MODULE_IMU, "Received %d init messages (adv: %s)",
        flushCount, gotAdvertisement ? "yes" : "no");

  // ✅ Si pas d'advertisement, envoyer INITIALIZE
  if (!gotAdvertisement) {
    LOG_I(LOG_MODULE_IMU, "Sending INITIALIZE command...");
    shtpData[0] = 0x04;  // INITIALIZE
    shtpData[1] = 0x00;
    if (sendPacket(CHANNEL_CONTROL, 2)) {
      delay(500);
      // Lire réponse
      for (int i = 0; i < 10; i++) {
        if (receivePacket()) {
          LOG_I(LOG_MODULE_IMU, "Init response: ch=%d, report=0x%02X",
                shtpHeader[2], (packetLength > 4) ? shtpData[0] : 0);
        }
        delay(50);
      }
    }
  }

  return true;
}

bool BNO08x_ESP32_P4::waitForI2C() {
  i2c_bus_id_t bus_id = (_i2c_bus == 1) ? I2C_BUS_1 : I2C_BUS_0;

  if (!i2c_lock(bus_id, 100)) {
    return false;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (_i2c_addr << 1) | I2C_MASTER_WRITE, false);
  i2c_master_stop(cmd);

  esp_err_t err = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);

  i2c_unlock(bus_id);

  return (err == ESP_OK || err == ESP_ERR_TIMEOUT);
}

bool BNO08x_ESP32_P4::dataAvailable() {
  if (receivePacket()) {
    if (shtpHeader[2] == CHANNEL_COMMAND && packetLength == 6) {
      LOG_W(LOG_MODULE_IMU, "Command response: 0x%02X 0x%02X",
            shtpData[0], shtpData[1]);
      return false;  // Ignorer ces messages
    }
    // Afficher ce qu'on reçoit pour debug
    LOG_I(LOG_MODULE_IMU, "Data: ch=%d, len=%d, report=0x%02X",
          shtpHeader[2], packetLength,
          (packetLength > 4) ? shtpData[0] : 0);

    if (shtpHeader[2] == CHANNEL_REPORTS || shtpHeader[2] == CHANNEL_WAKE_REPORTS) {
      LOG_I(LOG_MODULE_IMU, "Report ID: 0x%02X", shtpData[0]);
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

  // Lire le header
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (_i2c_addr << 1) | I2C_MASTER_READ, false);

  for (int i = 0; i < 3; i++) {
    i2c_master_read_byte(cmd, &shtpHeader[i], I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, &shtpHeader[3], I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  esp_err_t err = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(20));
  i2c_cmd_link_delete(cmd);

  if (err != ESP_OK) {
    i2c_unlock(bus_id);
    return false;
  }

  // Parser la taille
  packetLength = shtpHeader[0] | ((uint16_t)(shtpHeader[1] & 0x7F) << 8);

  // Vérifications
  if (packetLength == 0 || packetLength > MAX_PACKET_SIZE) {
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
  txPacket[3] = 0;

  memcpy(&txPacket[4], shtpData, dataLength);

  if (!i2c_lock(bus_id, 500)) {  // Timeout plus long
    return false;
  }

  LOG_I(LOG_MODULE_IMU, "Sending %d bytes on ch=%d: [%02X %02X %02X %02X...]",
        totalLength, channelNumber,
        txPacket[0], txPacket[1], txPacket[2], txPacket[3]);

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (_i2c_addr << 1) | I2C_MASTER_WRITE, true);  // CHECK ACK
  i2c_master_write(cmd, txPacket, totalLength, true);                     // Bloc complet
  i2c_master_stop(cmd);

  esp_err_t err = i2c_master_cmd_begin(_i2c_port, cmd, pdMS_TO_TICKS(100));  // 500ms
  i2c_cmd_link_delete(cmd);
  i2c_unlock(bus_id);

  if (err != ESP_OK) {
    LOG_W(LOG_MODULE_IMU, "Send failed: %s", esp_err_to_name(err));
    return false;
  }

  delay(200);  // BNO080 TRÈS lent à traiter
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

  bool success = false;
  for (int retry = 0; retry < 3; retry++) {
    // ✅ Reconstruire la commande à CHAQUE essai
    setFeatureCommand(0x05, timeBetweenReports);

    if (sendPacket(CHANNEL_CONTROL, 17)) {
      LOG_I(LOG_MODULE_IMU, "Sent SET_FEATURE: report=0x05, interval=%dms",
            timeBetweenReports);

      delay(500);

      // Chercher confirmation
      for (int i = 0; i < 10; i++) {
        if (receivePacket()) {
          LOG_I(LOG_MODULE_IMU, "Got: ch=%d, report=0x%02X",
                shtpHeader[2], (packetLength > 4) ? shtpData[0] : 0);

          if (shtpHeader[2] == CHANNEL_CONTROL) {
            success = true;
            break;
          }
        }
        delay(50);
      }
      if (success) break;
    }
    if (retry < 2) {
      LOG_W(LOG_MODULE_IMU, "Retry %d enabling feature", retry + 1);
      delay(500);
    }
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
float BNO08x_ESP32_P4::getQuatI() {
  return quatI;
}
float BNO08x_ESP32_P4::getQuatJ() {
  return quatJ;
}
float BNO08x_ESP32_P4::getQuatK() {
  return quatK;
}
float BNO08x_ESP32_P4::getQuatReal() {
  return quatReal;
}
float BNO08x_ESP32_P4::getQuatRadianAccuracy() {
  return quatRadianAccuracy;
}

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