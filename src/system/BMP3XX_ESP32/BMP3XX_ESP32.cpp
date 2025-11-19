/*!
 * @file BMP3XX.cpp
 *
 * BMP3XX temperature & barometric pressure sensor driver
 * Modified for ESP32-S3 new I2C driver (ESP-IDF v5.x+)
 *
 * Based on Bosch BMP3 sensor driver
 */

#include "BMP3XX_ESP32.h"
#include "esp_log.h"
#include <string.h>
#include <cmath>

static const char* TAG = "BMP3XX";

//#define BMP3XX_DEBUG

// Global handles for I2C communication
static i2c_master_dev_handle_t g_i2c_dev_handle = nullptr;

// Forward declarations
static int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
static int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
static void delay_usec(uint32_t us, void *intf_ptr);
static int8_t validate_trimming_param(struct bmp3_dev *dev);
static int8_t cal_crc(uint8_t seed, uint8_t data);

/***************************************************************************
 * PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
 *   @brief  Instantiates sensor
 */
/**************************************************************************/
BMP3XX_ESP32::BMP3XX_ESP32(void) {
    _meas_end = 0;
    _filterEnabled = _tempOSEnabled = _presOSEnabled = _ODREnabled = false;
    i2c_dev_handle = nullptr;
    i2c_bus_handle = nullptr;
    _own_bus = false;
}

/**************************************************************************/
/*!
 *   @brief  Destructor - cleanup resources
 */
/**************************************************************************/
BMP3XX_ESP32::~BMP3XX_ESP32() {
    if (i2c_dev_handle != nullptr) {
        i2c_master_bus_rm_device(i2c_dev_handle);
        i2c_dev_handle = nullptr;
    }

    // Only delete the bus if we created it
    if (_own_bus && i2c_bus_handle != nullptr) {
        i2c_del_master_bus(i2c_bus_handle);
        i2c_bus_handle = nullptr;
    }
}

/**************************************************************************/
/*!
 *   @brief Initializes the sensor with ESP32-S3 new I2C driver
 *
 *   @param  addr I2C address of BMP3. Default is 0x77
 *   @param  bus_handle I2C bus handle (must be initialized by user)
 *   @param  i2c_speed_hz I2C clock speed in Hz (default 400kHz)
 *   @return True on sensor initialization success. False on failure.
 */
/**************************************************************************/
bool BMP3XX_ESP32::begin_I2C(uint8_t addr, i2c_master_bus_handle_t bus_handle, uint32_t i2c_speed_hz) {
    // Clean up any existing device handle
    if (i2c_dev_handle != nullptr) {
        i2c_master_bus_rm_device(i2c_dev_handle);
        i2c_dev_handle = nullptr;
    }

    // Check if bus handle is provided
    if (bus_handle == nullptr) {
        ESP_LOGE(TAG, "Bus handle is NULL. Use begin_I2C_legacy() or provide valid bus handle.");
        return false;
    }

    // Store bus handle
    i2c_bus_handle = bus_handle;
    _i2caddr = addr;
    _own_bus = false; // We didn't create the bus

    // Configure device on I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = i2c_speed_hz,
    };

    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &i2c_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        return false;
    }

    // Set global handle for callback functions
    g_i2c_dev_handle = i2c_dev_handle;

    // Configure sensor structure
    the_sensor.chip_id = addr;
    the_sensor.intf = BMP3_I2C_INTF;
    the_sensor.read = &i2c_read;
    the_sensor.write = &i2c_write;
    the_sensor.intf_ptr = &g_i2c_dev_handle;
    the_sensor.dummy_byte = 0;
    the_sensor.delay_us = delay_usec;

    return _init();
}

/**************************************************************************/
/*!
 *   @brief Legacy I2C initialization (creates own bus)
 *
 *   @param  addr I2C address
 *   @param  sda_pin SDA GPIO pin
 *   @param  scl_pin SCL GPIO pin
 *   @param  i2c_speed_hz I2C speed in Hz
 *   @return True on success
 */
/**************************************************************************/
bool BMP3XX_ESP32::begin_I2C_legacy(uint8_t addr, int sda_pin, int scl_pin, uint32_t i2c_speed_hz) {
    // Create I2C bus configuration
    i2c_master_bus_config_t bus_config = {};
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.scl_io_num = (gpio_num_t)scl_pin;
    bus_config.sda_io_num = (gpio_num_t)sda_pin;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    // Create bus handle
    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        return false;
    }

    _own_bus = true; // We created the bus, so we'll clean it up

    return begin_I2C(addr, bus_handle, i2c_speed_hz);
}

/**************************************************************************/
/*!
 *   @brief Internal initialization function
 */
/**************************************************************************/
bool BMP3XX_ESP32::_init(void) {
    int8_t rslt = BMP3_OK;

    /* Reset the sensor */
    rslt = bmp3_soft_reset(&the_sensor);
    #ifdef BMP3XX_DEBUG
    ESP_LOGI(TAG, "Reset result: %d", rslt);
    #endif
    if (rslt != BMP3_OK)
        return false;

    rslt = bmp3_init(&the_sensor);
    #ifdef BMP3XX_DEBUG
    ESP_LOGI(TAG, "Init result: %d", rslt);
    #endif

    rslt = validate_trimming_param(&the_sensor);
    #ifdef BMP3XX_DEBUG
    ESP_LOGI(TAG, "Valtrim result: %d", rslt);
    #endif

    if (rslt != BMP3_OK)
        return false;

    #ifdef BMP3XX_DEBUG
    ESP_LOGI(TAG, "T1 = %u", the_sensor.calib_data.reg_calib_data.par_t1);
    ESP_LOGI(TAG, "T2 = %u", the_sensor.calib_data.reg_calib_data.par_t2);
    ESP_LOGI(TAG, "T3 = %d", the_sensor.calib_data.reg_calib_data.par_t3);
    ESP_LOGI(TAG, "P1 = %d", the_sensor.calib_data.reg_calib_data.par_p1);
    ESP_LOGI(TAG, "P2 = %d", the_sensor.calib_data.reg_calib_data.par_p2);
    ESP_LOGI(TAG, "P3 = %d", the_sensor.calib_data.reg_calib_data.par_p3);
    ESP_LOGI(TAG, "P4 = %d", the_sensor.calib_data.reg_calib_data.par_p4);
    ESP_LOGI(TAG, "P5 = %u", the_sensor.calib_data.reg_calib_data.par_p5);
    #endif

    setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    setPressureOversampling(BMP3_NO_OVERSAMPLING);
    setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
    setOutputDataRate(BMP3_ODR_25_HZ);

    // don't do anything till we request a reading
    the_sensor.settings.op_mode = BMP3_MODE_FORCED;

    return true;
}

/**************************************************************************/
/*!
 *   @brief Performs a reading and returns the ambient temperature.
 *   @return Temperature in degrees Centigrade
 */
/**************************************************************************/
float BMP3XX_ESP32::readTemperature(void) {
    performReading();
    return temperature;
}

/**************************************************************************/
/*!
 *   @brief Reads the chip identifier
 *   @return BMP3_CHIP_ID or BMP390_CHIP_ID
 */
/**************************************************************************/
uint8_t BMP3XX_ESP32::chipID(void) {
    return the_sensor.chip_id;
}

/**************************************************************************/
/*!
 *   @brief Performs a reading and returns the barometric pressure.
 *   @return Barometic pressure in Pascals
 */
/**************************************************************************/
float BMP3XX_ESP32::readPressure(void) {
    performReading();
    return pressure;
}

/**************************************************************************/
/*!
 *   @brief Calculates the altitude (in meters).
 *   @param  seaLevel Sea-level pressure in hPa
 *   @return Altitude in meters
 */
/**************************************************************************/
float BMP3XX_ESP32::readAltitude(float seaLevel) {
    float atmospheric = readPressure() / 100.0F;
    return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
 *   @brief Performs a full reading of all sensors in the BMP3XX.
 *   @return True on success, False on failure
 */
/**************************************************************************/
bool BMP3XX_ESP32::performReading(void) {
    int8_t rslt;
    uint16_t settings_sel = 0;
    uint8_t sensor_comp = 0;

    /* Select the pressure and temperature sensor to be enabled */
    the_sensor.settings.temp_en = BMP3_ENABLE;
    settings_sel |= BMP3_SEL_TEMP_EN;
    sensor_comp |= BMP3_TEMP;
    if (_tempOSEnabled) {
        settings_sel |= BMP3_SEL_TEMP_OS;
    }

    the_sensor.settings.press_en = BMP3_ENABLE;
    settings_sel |= BMP3_SEL_PRESS_EN;
    sensor_comp |= BMP3_PRESS;
    if (_presOSEnabled) {
        settings_sel |= BMP3_SEL_PRESS_OS;
    }

    if (_filterEnabled) {
        settings_sel |= BMP3_SEL_IIR_FILTER;
    }

    if (_ODREnabled) {
        settings_sel |= BMP3_SEL_ODR;
    }

    /* Set the desired sensor configuration */
    #ifdef BMP3XX_DEBUG
    ESP_LOGI(TAG, "Setting sensor settings");
    #endif
    rslt = bmp3_set_sensor_settings(settings_sel, &the_sensor);

    if (rslt != BMP3_OK)
        return false;

    /* Set the power mode */
    the_sensor.settings.op_mode = BMP3_MODE_FORCED;
    #ifdef BMP3XX_DEBUG
    ESP_LOGI(TAG, "Setting power mode");
    #endif
    rslt = bmp3_set_op_mode(&the_sensor);
    if (rslt != BMP3_OK)
        return false;

    /* Variable used to store the compensated data */
    struct bmp3_data data;

    /* Temperature and Pressure data are read and stored in the bmp3_data instance */
    #ifdef BMP3XX_DEBUG
    ESP_LOGI(TAG, "Getting sensor data");
    #endif
    rslt = bmp3_get_sensor_data(sensor_comp, &data, &the_sensor);
    if (rslt != BMP3_OK)
        return false;

    /* Save the temperature and pressure data */
    temperature = data.temperature;
    pressure = data.pressure;

    return true;
}

/**************************************************************************/
/*!
 *   @brief  Setter for Temperature oversampling
 *   @param  oversample Oversampling setting
 *   @return True on success, False on failure
 */
/**************************************************************************/
bool BMP3XX_ESP32::setTemperatureOversampling(uint8_t oversample) {
    if (oversample > BMP3_OVERSAMPLING_32X)
        return false;

    the_sensor.settings.odr_filter.temp_os = oversample;

    if (oversample == BMP3_NO_OVERSAMPLING)
        _tempOSEnabled = false;
    else
        _tempOSEnabled = true;

    return true;
}

/**************************************************************************/
/*!
 *   @brief  Setter for Pressure oversampling
 *   @param  oversample Oversampling setting
 *   @return True on success, False on failure
 */
/**************************************************************************/
bool BMP3XX_ESP32::setPressureOversampling(uint8_t oversample) {
    if (oversample > BMP3_OVERSAMPLING_32X)
        return false;

    the_sensor.settings.odr_filter.press_os = oversample;

    if (oversample == BMP3_NO_OVERSAMPLING)
        _presOSEnabled = false;
    else
        _presOSEnabled = true;

    return true;
}

/**************************************************************************/
/*!
 *   @brief  Setter for IIR filter coefficient
 *   @param filtercoeff Coefficient of the filter (in samples)
 *   @return True on success, False on failure
 */
/**************************************************************************/
bool BMP3XX_ESP32::setIIRFilterCoeff(uint8_t filtercoeff) {
    if (filtercoeff > BMP3_IIR_FILTER_COEFF_127)
        return false;

    the_sensor.settings.odr_filter.iir_filter = filtercoeff;

    if (filtercoeff == BMP3_IIR_FILTER_DISABLE)
        _filterEnabled = false;
    else
        _filterEnabled = true;

    return true;
}

/**************************************************************************/
/*!
 *   @brief  Setter for output data rate (ODR)
 *   @param odr Sample rate in Hz
 *   @return True on success, False on failure
 */
/**************************************************************************/
bool BMP3XX_ESP32::setOutputDataRate(uint8_t odr) {
    if (odr > BMP3_ODR_0_001_HZ)
        return false;

    the_sensor.settings.odr_filter.odr = odr;
    _ODREnabled = true;

    return true;
}

/**************************************************************************/
/*!
 *   @brief  Reads 8 bit values over I2C using new ESP32-S3 driver
 */
/**************************************************************************/
int8_t i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    i2c_master_dev_handle_t *dev_handle = (i2c_master_dev_handle_t *)intf_ptr;

    if (*dev_handle == nullptr) {
        ESP_LOGE(TAG, "I2C device handle is NULL");
        return 1;
    }

    esp_err_t err = i2c_master_transmit_receive(*dev_handle, &reg_addr, 1,
                                                reg_data, len, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
        return 1;
    }

    return 0;
}

/**************************************************************************/
/*!
 *   @brief  Writes 8 bit values over I2C using new ESP32-S3 driver
 */
/**************************************************************************/
int8_t i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    i2c_master_dev_handle_t *dev_handle = (i2c_master_dev_handle_t *)intf_ptr;

    if (*dev_handle == nullptr) {
        ESP_LOGE(TAG, "I2C device handle is NULL");
        return 1;
    }

    // Create write buffer with register address + data
    uint8_t *write_buf = (uint8_t *)malloc(len + 1);
    if (write_buf == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate write buffer");
        return 1;
    }

    write_buf[0] = reg_addr;
    memcpy(&write_buf[1], reg_data, len);

    esp_err_t err = i2c_master_transmit(*dev_handle, write_buf, len + 1, 1000);

    free(write_buf);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(err));
        return 1;
    }

    return 0;
}

/**************************************************************************/
/*!
 *   @brief  Delay function in microseconds
 */
/**************************************************************************/
static void delay_usec(uint32_t us, void *intf_ptr) {
    esp_rom_delay_us(us);
}

/**************************************************************************/
/*!
 *   @brief  Validate trimming parameters with CRC
 */
/**************************************************************************/
static int8_t validate_trimming_param(struct bmp3_dev *dev) {
    int8_t rslt;
    uint8_t crc = 0xFF;
    uint8_t stored_crc;
    uint8_t trim_param[21];
    uint8_t i;

    rslt = bmp3_get_regs(BMP3_REG_CALIB_DATA, trim_param, 21, dev);
    if (rslt == BMP3_OK) {
        for (i = 0; i < 21; i++) {
            crc = (uint8_t)cal_crc(crc, trim_param[i]);
        }

        crc = (crc ^ 0xFF);
        rslt = bmp3_get_regs(0x30, &stored_crc, 1, dev);
        if (stored_crc != crc) {
            rslt = -1;
        }
    }

    return rslt;
}

/**************************************************************************/
/*!
 *   @brief  Calculate CRC for trimming parameters
 */
/**************************************************************************/
static int8_t cal_crc(uint8_t seed, uint8_t data) {
    int8_t poly = 0x1D;
    int8_t var2;
    uint8_t i;

    for (i = 0; i < 8; i++) {
        if ((seed & 0x80) ^ (data & 0x80)) {
            var2 = 1;
        } else {
            var2 = 0;
        }

        seed = (seed & 0x7F) << 1;
        data = (data & 0x7F) << 1;
        seed = seed ^ (uint8_t)(poly * var2);
    }

    return (int8_t)seed;
}
