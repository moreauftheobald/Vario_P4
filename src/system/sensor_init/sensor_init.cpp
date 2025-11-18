/**
 * @file sensor_init.cpp
 * @brief Implémentation initialisation capteurs
 * 
 * @author Franck Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#include "sensor_init.h"
#include "src/system/logger/logger.h"
#include "config/config.h"  
#include "config/pins.h"

// Instances globales des capteurs
Adafruit_LSM6DSO32 lsm6dso32;
Adafruit_BMP5xx bmp585;
Adafruit_GPS gps(&Wire1);  // GPS sur I2C via Wire1
TwoWire* gps_i2c = &Wire1;

// Status capteurs
bool sensor_lsm6dso32_ready = false;
bool sensor_bmp585_ready = false;
bool sensor_gps_ready = false;

/**
 * @brief Initialise le bus I2C
 */
bool sensor_init_i2c() {
    LOG_I(LOG_MODULE_SYSTEM, "Initializing I2C1 bus...");
    
    // IMPORTANT : Utiliser le NOUVEAU driver I2C (compatible avec ESP_Panel)
    // Wire1.begin() utilise l'ancien driver par défaut
    // On doit forcer le nouveau driver avec setPins() puis begin()
    
    Wire1.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
    if (!Wire1.begin()) {
        LOG_E(LOG_MODULE_SYSTEM, "Failed to initialize I2C1");
        return false;
    }
    
    Wire1.setClock(I2C_FREQUENCY);
    Wire1.setTimeout(I2C_TIMEOUT_MS);
    
    // Petit délai pour stabilisation
    delay(100);
    
    LOG_I(LOG_MODULE_SYSTEM, "I2C1 initialized: SDA=%d SCL=%d freq=%d Hz",
          I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY);
    
    return true;
}

/**
 * @brief Scan le bus I2C
 */
uint8_t sensor_scan_i2c() {
    LOG_I(LOG_MODULE_SYSTEM, "Scanning I2C bus...");
    
    uint8_t count = 0;
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire1.beginTransmission(addr);
        uint8_t error = Wire1.endTransmission();
        
        if (error == 0) {
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

/**
 * @brief Initialise le LSM6DSO32
 */
bool sensor_init_lsm6dso32() {
    LOG_I(LOG_MODULE_SYSTEM, "Initializing LSM6DSO32...");
    LOG_V(LOG_MODULE_SYSTEM, "Configuring LSM6DSO32...");
    
    // Tentative d'initialisation
    if (!lsm6dso32.begin_I2C(LSM6DSO32_I2C_ADDR, &Wire1)) {
        LOG_E(LOG_MODULE_SYSTEM, "LSM6DSO32 not found at 0x%02X", LSM6DSO32_I2C_ADDR);
        sensor_lsm6dso32_ready = false;
        return false;
    }
    
    // Configuration accéléromètre
    lsm6dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE);
    lsm6dso32.setAccelDataRate(LSM6DSO32_ACCEL_ODR);
    
    // Configuration gyroscope
    lsm6dso32.setGyroRange(LSM6DSO32_GYRO_RANGE);
    lsm6dso32.setGyroDataRate(LSM6DSO32_GYRO_ODR);
    
    // Configuration filtres
    // Note: Les constantes de filtre sont des enums Adafruit, 
    // elles sont utilisées telles quelles depuis config.h
    
    // Extraire les valeurs pour affichage
    int accel_range = 8;  // ±8G
    int gyro_range = 125; // ±125°/s
    
    LOG_I(LOG_MODULE_SYSTEM, "LSM6DSO32 initialized: ±%dG, ±%d°/s @ 208Hz",
          accel_range, gyro_range);
    
    sensor_lsm6dso32_ready = true;
    return true;
}

/**
 * @brief Initialise le BMP585
 */
bool sensor_init_bmp585() {
    LOG_I(LOG_MODULE_SYSTEM, "Initializing BMP585...");
    LOG_V(LOG_MODULE_SYSTEM, "Configuring BMP585...");
    
    // Tentative d'initialisation
    if (!bmp585.begin(BMP585_I2C_ADDR, &Wire1)) {
        LOG_E(LOG_MODULE_SYSTEM, "BMP585 not found at 0x%02X", BMP585_I2C_ADDR);
        sensor_bmp585_ready = false;
        return false;
    }
    
    // Configuration oversampling
    bmp585.setTemperatureOversampling(BMP585_TEMP_OVERSAMPLE);
    bmp585.setPressureOversampling(BMP585_PRESS_OVERSAMPLE);
    
    // Configuration filtre IIR
    bmp585.setIIRFilterCoeff(BMP585_IIR_FILTER);
    
    // Configuration fréquence de sortie
    bmp585.setOutputDataRate(BMP585_OUTPUT_DATA_RATE);
    
    LOG_I(LOG_MODULE_SYSTEM, "BMP585 initialized: 8x temp, 32x press @ 50Hz");
    
    sensor_bmp585_ready = true;
    return true;
}

/**
 * @brief Initialise le GPS PA1010D
 */
bool sensor_init_gps() {
    LOG_I(LOG_MODULE_SYSTEM, "Initializing GPS PA1010D...");
    
    // Le GPS PA1010D est déjà sur Wire1 (initialisé dans le constructeur)
    // Tentative de communication
    if (!gps.begin(GPS_I2C_ADDR)) {
        LOG_E(LOG_MODULE_SYSTEM, "GPS PA1010D not found at 0x%02X", GPS_I2C_ADDR);
        sensor_gps_ready = false;
        return false;
    }
    
    // Configuration GPS
    // Activer sentences RMC (Recommended Minimum) et GGA (Fix data)
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    delay(100);
    
    // Fréquence d'update 1 Hz
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    delay(100);
    
    // Demander info antenne
    gps.sendCommand(PGCMD_ANTENNA);
    delay(100);
    
    LOG_I(LOG_MODULE_SYSTEM, "GPS PA1010D initialized @ 1Hz");
    LOG_I(LOG_MODULE_SYSTEM, "Waiting for GPS fix...");
    
    sensor_gps_ready = true;
    return true;
}

/**
 * @brief Initialise tous les capteurs
 */
bool sensor_init_all() {
    LOG_I(LOG_MODULE_SYSTEM, "=== Sensor Initialization ===");
    
    // 1. Initialiser I2C
    if (!sensor_init_i2c()) {
        LOG_E(LOG_MODULE_SYSTEM, "I2C initialization failed");
        return false;
    }
    
    // 2. Scanner le bus I2C (debug)
    uint8_t devices_found = sensor_scan_i2c();
    if (devices_found == 0) {
        LOG_E(LOG_MODULE_SYSTEM, "No I2C devices found!");
        return false;
    }
    
    // 3. Initialiser LSM6DSO32 (critique)
    sensor_init_lsm6dso32();
    
    // 4. Initialiser BMP585 (critique)
    sensor_init_bmp585();
    
    // 5. Initialiser GPS (non critique)
    sensor_init_gps();
    
    // 6. Afficher résumé
    sensor_init_print_summary();
    
    // 7. Vérifier capteurs critiques
    if (!sensor_check_critical()) {
        LOG_E(LOG_MODULE_SYSTEM, "Critical sensors failed!");
        return false;
    }
    
    LOG_I(LOG_MODULE_SYSTEM, "=== Sensor Init Complete ===");
    return true;
}

/**
 * @brief Affiche le résumé d'initialisation
 */
void sensor_init_print_summary() {
    LOG_I(LOG_MODULE_SYSTEM, "");
    LOG_I(LOG_MODULE_SYSTEM, "--- Sensor Summary ---");
    LOG_I(LOG_MODULE_SYSTEM, "LSM6DSO32 (IMU)    : %s", 
          sensor_lsm6dso32_ready ? "✓ OK" : "✗ FAIL");
    LOG_I(LOG_MODULE_SYSTEM, "BMP585 (Baro)      : %s", 
          sensor_bmp585_ready ? "✓ OK" : "✗ FAIL");
    LOG_I(LOG_MODULE_SYSTEM, "PA1010D (GPS)      : %s", 
          sensor_gps_ready ? "✓ OK" : "✗ FAIL");
    LOG_I(LOG_MODULE_SYSTEM, "----------------------");
    
    // Compter capteurs OK
    int ok_count = 0;
    if (sensor_lsm6dso32_ready) ok_count++;
    if (sensor_bmp585_ready) ok_count++;
    if (sensor_gps_ready) ok_count++;
    
    if (ok_count == 3) {
        LOG_I(LOG_MODULE_SYSTEM, "All sensors initialized successfully");
    } else if (ok_count > 0) {
        LOG_W(LOG_MODULE_SYSTEM, "Some sensors failed to initialize (%d/3 OK)", ok_count);
    } else {
        LOG_E(LOG_MODULE_SYSTEM, "No sensors initialized!");
    }
    
    LOG_I(LOG_MODULE_SYSTEM, "");
}

/**
 * @brief Vérifie si capteurs critiques sont OK
 */
bool sensor_check_critical() {
    // LSM6DSO32 et BMP585 sont critiques pour le vario
    // GPS est optionnel (vario fonctionne sans)
    
    if (!sensor_lsm6dso32_ready) {
        LOG_E(LOG_MODULE_SYSTEM, "Critical: LSM6DSO32 required for flight!");
        return false;
    }
    
    if (!sensor_bmp585_ready) {
        LOG_E(LOG_MODULE_SYSTEM, "Critical: BMP585 required for altitude!");
        return false;
    }
    
    if (!sensor_gps_ready) {
        LOG_W(LOG_MODULE_SYSTEM, "GPS not available - limited functionality");
    }
    
    return true;
}