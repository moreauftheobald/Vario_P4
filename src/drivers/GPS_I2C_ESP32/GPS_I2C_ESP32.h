#ifndef GPS_I2C_ESP32_H
#define GPS_I2C_ESP32_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
    #endif

    // Constantes
    #define GPS_I2C_DEFAULT_ADDR 0x10
    #define GPS_I2C_MAX_TRANSFER 32
    #define GPS_I2C_MAXLINELENGTH 120
    #define GPS_I2C_BUFFER_SIZE 256

    // Commandes PMTK
    #define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F"
    #define PMTK_SET_NMEA_UPDATE_5HZ "$PMTK220,200*2C"
    #define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
    #define PMTK_API_SET_FIX_CTL_1HZ "$PMTK300,1000,0,0,0,0*1C"
    #define PMTK_API_SET_FIX_CTL_5HZ "$PMTK300,200,0,0,0,0*2F"
    #define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
    #define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
    #define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
    #define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
    #define PMTK_Q_RELEASE "$PMTK605*31"
    #define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
    #define PMTK_STANDBY "$PMTK161,0*28"
    #define PMTK_AWAKE "$PMTK010,002*2D"

    // Configuration I2C
    typedef struct {
        uint8_t i2c_addr;
        uint32_t i2c_speed_hz;
    } gps_i2c_esp32_config_t;

    // Structure principale GPS
    typedef struct {
        // I2C
        i2c_master_bus_handle_t bus_handle;
        i2c_master_dev_handle_t dev_handle;
        uint8_t i2c_addr;

        // Buffer circulaire
        uint8_t buffer[GPS_I2C_BUFFER_SIZE];
        uint16_t buffer_head;
        uint16_t buffer_tail;

        // Lignes NMEA
        char line1[GPS_I2C_MAXLINELENGTH];
        char line2[GPS_I2C_MAXLINELENGTH];
        char *currentline;
        char *lastline;
        uint8_t lineidx;
        bool recvdflag;
        uint32_t recvdTime;
        uint32_t sentTime;

        // Donnees de temps
        uint8_t hour;
        uint8_t minute;
        uint8_t seconds;
        uint16_t milliseconds;
        uint8_t year;
        uint8_t month;
        uint8_t day;

        // Donnees de position
        float latitude;
        float longitude;
        float latitudeDegrees;
        float longitudeDegrees;
        int32_t latitude_fixed;
        int32_t longitude_fixed;
        char lat;
        char lon;

        // Donnees d'altitude et vitesse
        float altitude;
        float geoidheight;
        float speed;
        float angle;
        float magvariation;
        char mag;

        // Qualite du signal
        float HDOP;
        float VDOP;
        float PDOP;
        bool fix;
        uint8_t fixquality;
        uint8_t fixquality_3d;
        uint8_t satellites;
        uint8_t antenna;

        // Timestamps
        uint32_t lastFix;
        uint32_t lastTime;
        uint32_t lastDate;
        uint32_t lastUpdate;

        // Status
        bool paused;
        bool inStandbyMode;
        uint8_t last_char;

    } gps_i2c_esp32_t;

    // Fonctions d'initialisation
    esp_err_t GPS_I2C_ESP32_init(gps_i2c_esp32_t *gps,
                                 i2c_master_bus_handle_t bus_handle,
                                 const gps_i2c_esp32_config_t *config);
    esp_err_t GPS_I2C_ESP32_deinit(gps_i2c_esp32_t *gps);

    // Fonctions de communication
    esp_err_t GPS_I2C_ESP32_send_command(gps_i2c_esp32_t *gps, const char *str);
    char GPS_I2C_ESP32_read(gps_i2c_esp32_t *gps);
    uint16_t GPS_I2C_ESP32_available(gps_i2c_esp32_t *gps);

    // Fonctions NMEA
    bool GPS_I2C_ESP32_new_nmea_received(gps_i2c_esp32_t *gps);
    char *GPS_I2C_ESP32_last_nmea(gps_i2c_esp32_t *gps);
    bool GPS_I2C_ESP32_parse(gps_i2c_esp32_t *gps, char *nmea);
    bool GPS_I2C_ESP32_wait_for_sentence(gps_i2c_esp32_t *gps, const char *wait4me,
                                         uint8_t max_wait, uint32_t timeout_ms);

    // Fonctions de controle
    void GPS_I2C_ESP32_pause(gps_i2c_esp32_t *gps, bool p);
    bool GPS_I2C_ESP32_standby(gps_i2c_esp32_t *gps);
    bool GPS_I2C_ESP32_wakeup(gps_i2c_esp32_t *gps);

    // Fonctions utilitaires
    float GPS_I2C_ESP32_seconds_since_fix(gps_i2c_esp32_t *gps);
    float GPS_I2C_ESP32_seconds_since_time(gps_i2c_esp32_t *gps);
    float GPS_I2C_ESP32_seconds_since_date(gps_i2c_esp32_t *gps);
    void GPS_I2C_ESP32_add_checksum(char *buff);

    // Fonctions d'acces aux donnees
    bool GPS_I2C_ESP32_has_fix(gps_i2c_esp32_t *gps);
    uint8_t GPS_I2C_ESP32_get_satellites(gps_i2c_esp32_t *gps);
    float GPS_I2C_ESP32_get_latitude(gps_i2c_esp32_t *gps);
    float GPS_I2C_ESP32_get_longitude(gps_i2c_esp32_t *gps);
    float GPS_I2C_ESP32_get_altitude(gps_i2c_esp32_t *gps);
    float GPS_I2C_ESP32_get_speed(gps_i2c_esp32_t *gps);
    float GPS_I2C_ESP32_get_course(gps_i2c_esp32_t *gps);
    float GPS_I2C_ESP32_get_hdop(gps_i2c_esp32_t *gps);

    #ifdef __cplusplus
}
#endif

#endif // GPS_I2C_ESP32_H
