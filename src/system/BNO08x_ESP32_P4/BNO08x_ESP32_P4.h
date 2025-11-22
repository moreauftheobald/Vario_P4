#ifndef _BNO08X_ESP32_P4_H
#define _BNO08X_ESP32_P4_H

#include <Arduino.h>
#include "driver/i2c.h"  // ✅ Ancienne API I2C
#include "driver/gpio.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#define BNO08X_I2CADDR_DEFAULT 0x4A
#define BNO08X_I2CADDR_ALT 0x4B

class BNO08x_ESP32_P4 {
public:
    /**
     * @brief Constructeur
     * @param reset_pin Pin GPIO pour reset hardware (-1 si non utilisé)
     */
    BNO08x_ESP32_P4(int8_t reset_pin = -1);
    ~BNO08x_ESP32_P4();

    /**
     * @brief Initialisation I2C sur un port existant
     * 
     * @param i2c_port Port I2C (I2C_NUM_0 ou I2C_NUM_1)
     * @param i2c_addr Adresse I2C (0x4A ou 0x4B)
     * @param sensor_id ID capteur (réservé, mettre 0)
     * @return true si succès
     */
    bool begin_I2C(i2c_port_t i2c_port,
                   uint8_t i2c_addr = BNO08X_I2CADDR_DEFAULT,
                   int32_t sensor_id = 0);

    /**
     * @brief Reset hardware du BNO080
     */
    void hardwareReset(void);

    /**
     * @brief Vérifie si un reset s'est produit
     * @return true si reset détecté (flag auto-cleared)
     */
    bool wasReset(void);

    /**
     * @brief Active un rapport capteur
     * 
     * @param sensor ID du capteur (SH2_ROTATION_VECTOR, etc.)
     * @param interval_us Intervalle de rapport (µs)
     * @return true si succès
     */
    bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000);

    /**
     * @brief Lit un événement capteur
     * 
     * @param value Structure à remplir avec données
     * @return true si nouvel événement disponible
     */
    bool getSensorEvent(sh2_SensorValue_t *value);

    /**
     * @brief Product IDs du BNO080
     */
    sh2_ProductIds_t prodIds;

private:
    bool _init(int32_t sensor_id);

    int8_t _reset_pin;
    uint8_t _i2c_addr;
    i2c_port_t _i2c_port;
    sh2_Hal_t _HAL;

    static BNO08x_ESP32_P4* _instance;

    // HAL callbacks
    static int hal_open(sh2_Hal_t *self);
    static void hal_close(sh2_Hal_t *self);
    static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
    static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
    static uint32_t hal_getTimeUs(sh2_Hal_t *self);
};

#endif // _BNO08X_ESP32_P4_H