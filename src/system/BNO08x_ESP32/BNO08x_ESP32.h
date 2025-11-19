#ifndef _BNO08X_ESP32_H
#define _BNO08X_ESP32_H

#include "Arduino.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#define BNO08X_I2CADDR_DEFAULT 0x4A

class BNO08x_ESP32 {
public:
    BNO08x_ESP32(int8_t reset_pin = -1);
    ~BNO08x_ESP32();

    bool begin_I2C(i2c_master_bus_handle_t bus_handle,
                   uint8_t i2c_addr = BNO08X_I2CADDR_DEFAULT,
                   uint32_t freq = 100000,
                   int32_t sensor_id = 0);

    void hardwareReset(void);
    bool wasReset(void);
    bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000);
    bool getSensorEvent(sh2_SensorValue_t *value);

    sh2_ProductIds_t prodIds;

private:
    bool _init(int32_t sensor_id);

    int8_t _reset_pin;
    uint8_t _i2c_addr;
    i2c_master_bus_handle_t _bus_handle;
    i2c_master_dev_handle_t _dev_handle;
    sh2_Hal_t _HAL;
    bool _owns_bus;

    static BNO08x_ESP32* _instance;

    static int hal_open(sh2_Hal_t *self);
    static void hal_close(sh2_Hal_t *self);
    static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
    static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
    static uint32_t hal_getTimeUs(sh2_Hal_t *self);
};

#endif
