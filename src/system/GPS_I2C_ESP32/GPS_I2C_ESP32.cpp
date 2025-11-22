/**
 * @file GPS_I2C_ESP32.cpp
 * @brief Implémentation driver GPS PA1010D simplifié
 */

#include "GPS_I2C_ESP32.h"
#include "src/system/logger/logger.h"
#include <string.h>
#include <stdlib.h>
#include <driver/i2c.h>

// Déclaration des fonctions lock/unlock du wrapper
extern bool i2c_lock(i2c_bus_id_t bus, uint32_t timeout_ms);
extern void i2c_unlock(i2c_bus_id_t bus);


// ============================================================================
// I2C DIRECT
// ============================================================================
static i2c_port_t get_port(i2c_bus_id_t bus) {
  return (bus == I2C_BUS_1) ? I2C_NUM_1 : I2C_NUM_0;
}

// GPS_I2C_ESP32.cpp

static bool i2c_write_bytes(gps_device_t* dev, const uint8_t* data, size_t len) {
  if (!i2c_lock(dev->bus, I2C_TIMEOUT_MS)) {
    LOG_W(LOG_MODULE_GPS, "I2C lock timeout (write)");
    return false;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write(cmd, (uint8_t*)data, len, true);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(get_port(dev->bus), cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  i2c_unlock(dev->bus);

  return (ret == ESP_OK);
}

static bool i2c_read_bytes(gps_device_t* dev, uint8_t* data, size_t len) {
  if (!i2c_lock(dev->bus, I2C_TIMEOUT_MS)) {
    LOG_W(LOG_MODULE_GPS, "I2C lock timeout (read)");
    return false;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(get_port(dev->bus), cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  i2c_unlock(dev->bus);

  return (ret == ESP_OK);
}

// ============================================================================
// PARSING NMEA SIMPLIFIÉ
// ============================================================================

/**
 * @brief Vérifie checksum NMEA
 */
static bool check_nmea_checksum(const char* nmea) {
  if (!nmea || nmea[0] != '$') return false;

  const char* star = strchr(nmea, '*');
  if (!star) return false;

  // Calculer checksum (XOR entre $ et *)
  uint8_t checksum = 0;
  for (const char* p = nmea + 1; p < star; p++) {
    checksum ^= *p;
  }

  // Comparer avec checksum dans la phrase
  uint8_t expected = strtol(star + 1, NULL, 16);

  return (checksum == expected);
}

/**
 * @brief Parse coordonnée GPS (format ddmm.mmmm)
 */
static bool parse_coord(const char* str, char dir, float* degrees) {
  if (!str || !degrees) return false;

  // Format: ddmm.mmmm (latitude) ou dddmm.mmmm (longitude)
  float raw = atof(str);

  int deg = (int)(raw / 100);
  float min = raw - (deg * 100);

  *degrees = deg + (min / 60.0f);

  // Appliquer direction (S/W = négatif)
  if (dir == 'S' || dir == 'W') {
    *degrees = -*degrees;
  }

  return true;
}

/**
 * @brief Parse temps UTC (hhmmss.sss)
 */
static void parse_time(const char* str, gps_data_t* data) {
  if (!str || !data) return;

  uint32_t time = atol(str);
  data->hour = time / 10000;
  data->minute = (time % 10000) / 100;
  data->second = time % 100;

  // Millisecondes (après le point)
  const char* dot = strchr(str, '.');
  if (dot) {
    data->millisecond = atoi(dot + 1);
  }
}

/**
 * @brief Parse date UTC (ddmmyy)
 */
static void parse_date(const char* str, gps_data_t* data) {
  if (!str || !data) return;

  uint32_t date = atol(str);
  data->day = date / 10000;
  data->month = (date % 10000) / 100;
  data->year = date % 100;
}

/**
 * @brief Parse phrase GPRMC (position + vitesse + date)
 */
static bool parse_rmc(const char* nmea, gps_data_t* data) {
  // $GPRMC,hhmmss.sss,A,ddmm.mmmm,N,dddmm.mmmm,E,speed,course,ddmmyy,,,A*checksum

  char* tokens[15];
  char buffer[GPS_MAX_LINE_LEN];
  strncpy(buffer, nmea, sizeof(buffer));

  int count = 0;
  char* token = strtok(buffer, ",");
  while (token && count < 15) {
    tokens[count++] = token;
    token = strtok(NULL, ",");
  }

  if (count < 10) return false;

  // Token 2: status (A=active, V=void)
  data->fix = (tokens[2][0] == 'A');

  if (!data->fix) return false;

  // Token 1: time
  parse_time(tokens[1], data);

  // Token 3,4: latitude
  parse_coord(tokens[3], tokens[4][0], &data->latitude);

  // Token 5,6: longitude
  parse_coord(tokens[5], tokens[6][0], &data->longitude);

  // Token 7: speed (knots)
  data->speed = atof(tokens[7]);

  // Token 8: course (degrees)
  data->course = atof(tokens[8]);

  // Token 9: date
  parse_date(tokens[9], data);

  data->last_update_ms = millis();

  return true;
}

/**
 * @brief Parse phrase GPGGA (altitude + qualité)
 */
static bool parse_gga(const char* nmea, gps_data_t* data) {
  // $GPGGA,hhmmss.sss,ddmm.mmmm,N,dddmm.mmmm,E,fix,sats,hdop,alt,M,...

  char* tokens[15];
  char buffer[GPS_MAX_LINE_LEN];
  strncpy(buffer, nmea, sizeof(buffer));

  int count = 0;
  char* token = strtok(buffer, ",");
  while (token && count < 15) {
    tokens[count++] = token;
    token = strtok(NULL, ",");
  }

  if (count < 10) return false;

  // Token 6: fix quality (0=no fix, 1=GPS, 2=DGPS)
  int fix_quality = atoi(tokens[6]);
  data->fix = (fix_quality > 0);

  if (!data->fix) return false;

  // Token 7: satellites
  data->satellites = atoi(tokens[7]);

  // Token 8: HDOP
  data->hdop = atof(tokens[8]);

  // Token 9: altitude
  data->altitude = atof(tokens[9]);

  data->last_update_ms = millis();

  return true;
}

// ============================================================================
// INIT
// ============================================================================
bool GPS_init(gps_device_t* dev, const gps_i2c_config_t* config) {
  if (!dev || !config) return false;

  LOG_I(LOG_MODULE_GPS, "Initializing GPS PA1010D...");

  memset(dev, 0, sizeof(gps_device_t));
  dev->bus = config->bus;
  dev->address = config->address;

  // Vérifier présence sur I2C
  if (!i2c_probe_device(config->bus, config->address)) {
    LOG_E(LOG_MODULE_GPS, "GPS not found at 0x%02X", config->address);
    return false;
  }

  delay(100);

  // Configurer output RMC + GGA
  GPS_send_command(dev, PMTK_SET_NMEA_OUTPUT_RMCGGA);
  delay(100);

  // Configurer update rate 1Hz
  GPS_send_command(dev, PMTK_SET_NMEA_UPDATE_1HZ);
  delay(100);

  dev->initialized = true;

  LOG_I(LOG_MODULE_GPS, "GPS initialized @ 1Hz (RMC+GGA)");

  return true;
}

/**
 * @brief Lit le nombre de bytes disponibles dans le buffer GPS
 * 
 * Le PA1010D expose 2 registres (0xFD/0xFE) indiquant le nombre
 * de bytes disponibles. Permet d'éviter les lectures inutiles.
 * 
 * @param[in] dev Device GPS
 * @return Nombre de bytes disponibles (0 si erreur ou vide)
 */
static uint16_t gps_get_available_bytes(gps_device_t* dev) {
  if (!dev) return 0;

  // Acquisition verrou
  if (!i2c_lock(dev->bus, I2C_TIMEOUT_MS)) {
    return 0;
  }

  uint8_t len_data[2];

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, GPS_REG_DATA_LEN_MSB, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, len_data, 2, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(get_port(dev->bus), cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  // Libération verrou
  i2c_unlock(dev->bus);

  if (ret != ESP_OK) {
    return 0;
  }

  // MSB first
  return ((uint16_t)len_data[0] << 8) | len_data[1];
}

// ============================================================================
// READ
// ============================================================================
bool GPS_read(gps_device_t* dev, gps_data_t* data) {
  if (!dev || !dev->initialized || !data) return false;

  // ✅ OPTIMISATION : Vérifier d'abord s'il y a des données disponibles
  uint16_t available = gps_get_available_bytes(dev);

  if (available == 0) {
    // Pas de données, économiser la transaction I2C
    return false;
  }

  // Lire seulement ce qui est disponible (max 32 bytes à la fois)
  uint8_t to_read = (available > GPS_I2C_MAX_TRANSFER) ? GPS_I2C_MAX_TRANSFER : available;

  // Allouer buffer dynamiquement selon besoin
  uint8_t buffer[GPS_I2C_MAX_TRANSFER];

  if (!i2c_read_bytes(dev, buffer, to_read)) {
    return false;
  }

  // Traiter chaque caractère
  for (int i = 0; i < to_read; i++) {
    char c = buffer[i];

    // Ignorer stray newlines
    if (c == '\n' && dev->last_char != '\r') {
      continue;
    }

    dev->last_char = c;

    // Fin de ligne ?
    if (c == '\n') {
      dev->line_buffer[dev->line_index] = '\0';
      dev->line_index = 0;

      // Vérifier checksum
      if (!check_nmea_checksum(dev->line_buffer)) {
        LOG_V(LOG_MODULE_GPS, "Invalid checksum: %s", dev->line_buffer);
        continue;
      }

      // Parser selon type
      if (strstr(dev->line_buffer, "RMC")) {
        if (parse_rmc(dev->line_buffer, data)) {
          LOG_V(LOG_MODULE_GPS, "RMC: %.6f,%.6f speed=%.1f",
                data->latitude, data->longitude, data->speed);
          return true;
        }
      } else if (strstr(dev->line_buffer, "GGA")) {
        if (parse_gga(dev->line_buffer, data)) {
          LOG_V(LOG_MODULE_GPS, "GGA: alt=%.1f sats=%d hdop=%.1f",
                data->altitude, data->satellites, data->hdop);
          return true;
        }
      }
    } else if (dev->line_index < GPS_MAX_LINE_LEN - 1) {
      dev->line_buffer[dev->line_index++] = c;
    }
  }

  return false;
}

// ============================================================================
// SEND COMMAND
// ============================================================================
bool GPS_send_command(gps_device_t* dev, const char* cmd) {
  if (!dev || !cmd) return false;

  // Ajouter CRLF
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "%s\r\n", cmd);

  LOG_V(LOG_MODULE_GPS, "Sending: %s", cmd);

  return i2c_write_bytes(dev, (const uint8_t*)buffer, strlen(buffer));
}