/*!
 * @file BMP3XX.h
 *
 * BMP3XX temperature & barometric pressure sensor driver
 * Modified for ESP32-S3 new I2C driver (ESP-IDF v5.x+)
 *
 * Based on Bosch BMP3 sensor driver
 * Supports BMP388 and BMP390
 */

#ifndef __BMP3XX_H__
#define __BMP3XX_H__

#include "bmp3.h"
#include "driver/i2c_master.h"

/*=========================================================================
 I 2*C ADDRESS/BITS
 -----------------------------------------------------------------------*/  
#define BMP3XX_DEFAULT_ADDRESS (0x77) ///< The default I2C address
#define BMP3XX_ALT_ADDRESS     (0x76) ///< Alternative I2C address
/*=========================================================================*/

/** BMP3XX Class for ESP32-S3 I2C usage.
 *  Uses ESP-IDF v5.x I2C Master driver
 */

class BMP3XX_ESP32 {
public:
    BMP3XX_ESP32();
    ~BMP3XX_ESP32();

    // I2C initialization with new ESP32-S3 driver
    bool begin_I2C(uint8_t addr = BMP3XX_DEFAULT_ADDRESS,
                   i2c_master_bus_handle_t bus_handle = nullptr,
                   uint32_t i2c_speed_hz = 400000);

    // Legacy I2C initialization (deprecated but kept for compatibility)
    bool begin_I2C_legacy(uint8_t addr, int sda_pin, int scl_pin,
                          uint32_t i2c_speed_hz = 400000);

    uint8_t chipID(void);
    float readTemperature(void);
    float readPressure(void);
    float readAltitude(float seaLevel);

    bool setTemperatureOversampling(uint8_t os);
    bool setPressureOversampling(uint8_t os);
    bool setIIRFilterCoeff(uint8_t fs);
    bool setOutputDataRate(uint8_t odr);

    /// Perform a reading in blocking mode
    bool performReading(void);

    /// Temperature (Celsius) assigned after calling performReading()
    double temperature;
    /// Pressure (Pascals) assigned after calling performReading()
    double pressure;

private:
    i2c_master_dev_handle_t i2c_dev_handle = nullptr;
    i2c_master_bus_handle_t i2c_bus_handle = nullptr;
    bool _own_bus = false; // Track if we created the bus

    bool _init(void);

    bool _filterEnabled, _tempOSEnabled, _presOSEnabled, _ODREnabled;
    uint8_t _i2caddr;
    int32_t _sensorID;
    unsigned long _meas_end;

    struct bmp3_dev the_sensor;
};

#endif
