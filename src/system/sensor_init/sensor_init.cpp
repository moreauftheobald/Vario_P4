/**
 * @file sensor_init.cpp
 * @brief Implémentation - VERSION GPS UNIQUEMENT
 *
 * Tout ce qui ne concerne pas le GPS a été commenté.
 * Utilisation de :
 *  - i2c_wrapper
 *  - GPS_I2C_ESP32
 *  - bus I2C déjà initialisé
 *
 * Auteur : Franck Moreau
 */

#include "src/system/sensor_init/sensor_init.h"
#include "src/system/logger/logger.h"
#include "config/config.h"
#include "src/system/GPS_I2C_ESP32/GPS_I2C_ESP32.h"

// ================================
// === COMMENTÉ : AUTRES CAPTEURS
// ================================
/*
#include <Adafruit_LSM6DSO32.h>
#include "Adafruit_BMP5xx.h"
*/


// ================================
// === GPS UNIQUEMENT
// ================================

GPS_I2C_ESP32 gps(I2C_BUS_1, GPS_I2C_ADDR);
bool sensor_gps_ready = false;


// ================================
// === COMMENTÉ : I2C INIT ORIGINAL
// ================================
/*
bool sensor_init_i2c() {
    LOG_I(LOG_MODULE_SYSTEM, "Initializing I2C1 bus...");
    return true;
}
*/


// ================================
// === SCAN I2C VIA WRAPPER
// ================================
uint8_t sensor_scan_i2c() {
    LOG_I(LOG_MODULE_SYSTEM, "Scanning I2C bus (wrapper)...");

    uint8_t count = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_probe_device(I2C_BUS_1, addr)) {
            LOG_V(LOG_MODULE_SYSTEM, "I2C device found at 0x%02X", addr);
            count++;
        }
    }

    LOG_I(LOG_MODULE_SYSTEM, "I2C scan found %d device(s)", count);

    if (count == 0) {
        LOG_W(LOG_MODULE_SYSTEM, "No I2C devices found - check wiring!");
    }

    return count;
}


// ================================
// === INIT GPS PA1010D
// ================================
bool sensor_init_gps() {
    LOG_I(LOG_MODULE_SYSTEM, "Initializing GPS PA1010D (I2C Wrapper)...");

    if (!gps.begin()) {
        LOG_E(LOG_MODULE_SYSTEM, "GPS PA1010D not found at 0x%02X", GPS_I2C_ADDR);
        sensor_gps_ready = false;
        return false;
    }

    // Configuration GPS
    gps.setUpdateRate(1);
    gps.enableNMEA(GPS_NMEA_RMC | GPS_NMEA_GGA);

    LOG_I(LOG_MODULE_SYSTEM, "GPS PA1010D initialized @ 1Hz");
    sensor_gps_ready = true;
    return true;
}


// ================================
// === INIT GLOBAL (GPS ONLY)
// ================================
bool sensor_init_all() {
    LOG_I(LOG_MODULE_SYSTEM, "=== GPS Initialization ===");

    // Scan I2C (debug)
    uint8_t devices_found = sensor_scan_i2c();
    if (devices_found == 0) {
        LOG_W(LOG_MODULE_SYSTEM, "No I2C devices found on GPS bus");
    }

    sensor_init_gps();

    sensor_init_print_summary();

    LOG_I(LOG_MODULE_SYSTEM, "=== GPS Init Complete ===");
    return sensor_gps_ready;
}


// ================================
// === SUMMARY GPS ONLY
// ================================
void sensor_init_print_summary() {
    LOG_I(LOG_MODULE_SYSTEM, "");
    LOG_I(LOG_MODULE_SYSTEM, "--- GPS Summary ---");
    LOG_I(LOG_MODULE_SYSTEM, "PA1010D (GPS) : %s",
          sensor_gps_ready ? "✓ OK" : "✗ FAIL");
    LOG_I(LOG_MODULE_SYSTEM, "------------------");
    LOG_I(LOG_MODULE_SYSTEM, "");
}
