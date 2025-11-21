#include "GPS_I2C_ESP32.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"  // nécessaire pour i2c_cmd / flags

static const char *TAG = "GPS_I2C_ESP32";

// -----------------------------------------------------------------------------
// Fonctions privées (prototypes)
// -----------------------------------------------------------------------------
static esp_err_t gps_i2c_write(gps_i2c_esp32_t *gps, const uint8_t *data, size_t len);
static esp_err_t gps_i2c_read(gps_i2c_esp32_t *gps, uint8_t *data, size_t len);
static void gps_common_init(gps_i2c_esp32_t *gps);
static bool gps_parse_coord(char *p, float *angleDegrees, float *angle, int32_t *angle_fixed, char *dir);
static bool gps_parse_time(gps_i2c_esp32_t *gps, char *p);
static bool gps_parse_fix(gps_i2c_esp32_t *gps, char *p);
static bool gps_parse_antenna(gps_i2c_esp32_t *gps, char *p);
static bool gps_is_empty(char *p);
static uint8_t gps_parse_hex(char c);
static bool gps_check_nmea(char *nmea);

// -----------------------------------------------------------------------------
// Mapping bus utilisateur -> port driver (local, simple)
// -----------------------------------------------------------------------------
static inline i2c_port_t gps_to_driver_port(i2c_bus_id_t bus) {
  return (bus == I2C_BUS_1) ? I2C_NUM_1 : I2C_NUM_0;
}

// ============================================================================
// INITIALISATION (version i2c_wrapper)
// ============================================================================

esp_err_t GPS_I2C_ESP32_init(gps_i2c_esp32_t *gps,
                             const gps_i2c_esp32_config_t *config) {
  if (!gps || !config) {
    return ESP_ERR_INVALID_ARG;
  }

  gps_common_init(gps);

  gps->bus = config->bus;
  gps->i2c_addr = config->i2c_addr;

#ifdef DEBUG_MODE
  ESP_LOGI(TAG, "GPS I2C init on bus %d addr 0x%02X", gps->bus, gps->i2c_addr);
#endif

  // Le bus est supposé déjà initialisé par ailleurs (dixit tes remarques).
  // Simple délai pour laisser le périphérique démarrer si nécessaire.
  vTaskDelay(pdMS_TO_TICKS(10));

  return ESP_OK;
}

esp_err_t GPS_I2C_ESP32_deinit(gps_i2c_esp32_t *gps) {
  if (!gps) return ESP_ERR_INVALID_ARG;

#ifdef DEBUG_MODE
  ESP_LOGI(TAG, "GPS I2C deinit");
#endif

  // Rien de spécifique à faire ici : on ne supprime pas le driver car il est
  // géré au niveau du wrapper/initialisation globale.
  return ESP_OK;
}

static void gps_common_init(gps_i2c_esp32_t *gps) {
  memset(gps, 0, sizeof(gps_i2c_esp32_t));

  gps->currentline = gps->line1;
  gps->lastline = gps->line2;
  gps->lineidx = 0;
  gps->recvdflag = false;
  gps->paused = false;
  gps->inStandbyMode = false;

  gps->buffer_head = 0;
  gps->buffer_tail = 0;

  gps->lat = 'X';
  gps->lon = 'X';
  gps->mag = 'X';
  gps->fix = false;

  gps->lastFix = 2000000000UL;
  gps->lastTime = 2000000000UL;
  gps->lastDate = 2000000000UL;
  gps->lastUpdate = 2000000000UL;
}

// ============================================================================
// COMMUNICATION I2C via i2c_wrapper (raw transactions with locking)
// ============================================================================

static esp_err_t gps_i2c_write(gps_i2c_esp32_t *gps, const uint8_t *data, size_t len) {
  if (!gps || !data || len == 0) return ESP_ERR_INVALID_ARG;

  // Acquire bus lock via wrapper
  if (!i2c_lock(gps->bus, I2C_TIMEOUT_MS)) {
#ifdef DEBUG_MODE
    ESP_LOGW(TAG, "i2c lock timeout (write) bus=%d", gps->bus);
#endif
    return ESP_ERR_TIMEOUT;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // Address + write bit
  i2c_master_write_byte(cmd, (gps->i2c_addr << 1) | I2C_MASTER_WRITE, true);
  // raw write of payload
  i2c_master_write(cmd, (uint8_t *)data, len, true);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(gps_to_driver_port(gps->bus), cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  i2c_unlock(gps->bus);

#ifdef DEBUG_MODE
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "I2C write failed (addr 0x%02X): %s", gps->i2c_addr, esp_err_to_name(ret));
  }
#endif

  return ret;
}

static esp_err_t gps_i2c_read(gps_i2c_esp32_t *gps, uint8_t *data, size_t len) {
  if (!gps || !data || len == 0) return ESP_ERR_INVALID_ARG;

  // Acquire bus lock via wrapper
  if (!i2c_lock(gps->bus, I2C_TIMEOUT_MS)) {
#ifdef DEBUG_MODE
    ESP_LOGW(TAG, "i2c lock timeout (read) bus=%d", gps->bus);
#endif
    return ESP_ERR_TIMEOUT;
  }

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  // Address + read bit
  i2c_master_write_byte(cmd, (gps->i2c_addr << 1) | I2C_MASTER_READ, true);

  // Read len bytes. Use LAST_NACK for the last byte (ESP-IDF helper)
  if (len > 0) {
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
  }
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(gps_to_driver_port(gps->bus), cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
  i2c_cmd_link_delete(cmd);

  i2c_unlock(gps->bus);

#ifdef DEBUG_MODE
  if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
    ESP_LOGE(TAG, "I2C read failed (addr 0x%02X): %s", gps->i2c_addr, esp_err_to_name(ret));
  }
#endif

  return ret;
}

// Wrapper for sending ASCII NMEA / PMTK commands
esp_err_t GPS_I2C_ESP32_send_command(gps_i2c_esp32_t *gps, const char *str) {
  if (!gps || !str) return ESP_ERR_INVALID_ARG;

  size_t len = strlen(str);
  // Add CRLF if not empty (like original)
  if (len == 0) {
    // Some modules may expect a wake sequence; send nothing
    return ESP_OK;
  }

  // Build command with CRLF
  char *cmd = (char *)malloc(len + 3);
  if (!cmd) return ESP_ERR_NO_MEM;
  strcpy(cmd, str);
  strcat(cmd, "\r\n");

  esp_err_t ret = gps_i2c_write(gps, (const uint8_t *)cmd, strlen(cmd));
  free(cmd);

#ifdef DEBUG_MODE
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Sent command: %s", str);
  }
#endif

  return ret;
}

// ============================================================================
// LECTURE DONNEES
// ============================================================================

uint16_t GPS_I2C_ESP32_available(gps_i2c_esp32_t *gps) {
  if (!gps || gps->paused) return 0;

  if (gps->buffer_head >= gps->buffer_tail) {
    return (uint16_t)(gps->buffer_head - gps->buffer_tail);
  } else {
    return (uint16_t)(GPS_I2C_BUFFER_SIZE - gps->buffer_tail + gps->buffer_head);
  }
}

char GPS_I2C_ESP32_read(gps_i2c_esp32_t *gps) {
  static uint32_t firstChar = 0;
  uint32_t tStart = xTaskGetTickCount() * portTICK_PERIOD_MS;
  char c = 0;

  if (!gps || gps->paused) return c;

  if (GPS_I2C_ESP32_available(gps) == 0) {
    uint8_t i2c_buffer[GPS_I2C_MAX_TRANSFER];

    // Try to read MAX_TRANSFER bytes from the GPS
    esp_err_t r = gps_i2c_read(gps, i2c_buffer, GPS_I2C_MAX_TRANSFER);
    if (r == ESP_OK) {
      for (int i = 0; i < GPS_I2C_MAX_TRANSFER; i++) {
        uint8_t curr_char = i2c_buffer[i];

        // handle stray newline logic from original code
        if ((curr_char == 0x0A) && (gps->last_char != 0x0D)) {
          continue;
        }

        gps->last_char = curr_char;

        uint16_t next_head = (uint16_t)((gps->buffer_head + 1) % GPS_I2C_BUFFER_SIZE);
        if (next_head != gps->buffer_tail) {
          gps->buffer[gps->buffer_head] = curr_char;
          gps->buffer_head = next_head;
        }
      }
    }
  }

  if (GPS_I2C_ESP32_available(gps) > 0) {
    c = (char)gps->buffer[gps->buffer_tail];
    gps->buffer_tail = (uint16_t)((gps->buffer_tail + 1) % GPS_I2C_BUFFER_SIZE);
  } else {
    return 0;
  }

  // Build current line
  gps->currentline[gps->lineidx++] = c;
  if (gps->lineidx >= GPS_I2C_MAXLINELENGTH) gps->lineidx = GPS_I2C_MAXLINELENGTH - 1;

  if (c == '\n') {
    gps->currentline[gps->lineidx] = 0;

    char *temp = gps->currentline;
    gps->currentline = gps->lastline;
    gps->lastline = temp;

    gps->lineidx = 0;
    gps->recvdflag = true;
    gps->recvdTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    gps->sentTime = firstChar;
    firstChar = 0;
    return c;
  }

  if (firstChar == 0) firstChar = tStart;
  return c;
}

// Rest of parsing functions left unchanged (copy from original)
// ... (we include the rest of your original parsing utilities exactly as they were)
// For brevity here I paste them unchanged:

// ------------------ helpers ------------------

static uint8_t gps_parse_hex(char c) {
  if (c < '0') return 0;
  if (c <= '9') return c - '0';
  if (c < 'A') return 0;
  if (c <= 'F') return (c - 'A') + 10;
  return 0;
}

static bool gps_is_empty(char *p) {
  if (!p) return true;
  return (*p == ',' || *p == '*');
}

static bool gps_check_nmea(char *nmea) {
  if (!nmea) return false;
  if (*nmea != '$' && *nmea != '!') return false;

  char *ast = strchr(nmea, '*');
  if (!ast) return false;

  uint16_t sum = gps_parse_hex(*(ast + 1)) * 16;
  sum += gps_parse_hex(*(ast + 2));

  for (char *p = nmea + 1; p < ast; p++) {
    sum ^= *p;
  }

  return (sum == 0);
}

static bool gps_parse_coord(char *p, float *angleDegrees, float *angle, int32_t *angle_fixed, char *dir) {
  if (gps_is_empty(p)) return false;

  char degreebuff[10] = { 0 };
  char *e = strchr(p, '.');

  if (!e || (e - p) > 6) return false;

  strncpy(degreebuff, p, e - p);
  long dddmm = atol(degreebuff);
  long degrees = dddmm / 100;
  long minutes = dddmm - degrees * 100;

  float decminutes = atof(e);
  p = strchr(p, ',') + 1;

  char nsew = 'X';
  if (!gps_is_empty(p)) {
    nsew = *p;
  } else {
    return false;
  }

  int32_t fixed = degrees * 10000000 + (minutes * 10000000) / 60 + (int32_t)(decminutes * 10000000) / 60;
  float ang = degrees * 100 + minutes + decminutes;
  float deg = fixed / 10000000.0f;

  if (nsew == 'S' || nsew == 'W') {
    fixed = -fixed;
    deg = -deg;
  }

  if (nsew != 'N' && nsew != 'S' && nsew != 'E' && nsew != 'W') return false;
  if ((nsew == 'N' || nsew == 'S') && fabs(deg) > 90) return false;
  if (fabs(deg) > 180) return false;

  if (angle) *angle = ang;
  if (angle_fixed) *angle_fixed = fixed;
  if (angleDegrees) *angleDegrees = deg;
  if (dir) *dir = nsew;

  return true;
}

static bool gps_parse_time(gps_i2c_esp32_t *gps, char *p) {
  if (gps_is_empty(p)) return false;

  uint32_t time = atol(p);
  gps->hour = time / 10000;
  gps->minute = (time % 10000) / 100;
  gps->seconds = (time % 100);

  char *dec = strchr(p, '.');
  char *comma = strchr(p, ',');
  char *ast = strchr(p, '*');
  char *comstar = (comma && ast) ? (comma < ast ? comma : ast) : (comma ? comma : ast);

  if (dec && comstar && dec < comstar) {
    gps->milliseconds = (uint16_t)(atof(dec) * 1000);
  } else {
    gps->milliseconds = 0;
  }

  gps->lastTime = gps->sentTime;
  return true;
}

static bool gps_parse_fix(gps_i2c_esp32_t *gps, char *p) {
  if (gps_is_empty(p)) return false;

  if (p[0] == 'A') {
    gps->fix = true;
    gps->lastFix = gps->sentTime;
  } else if (p[0] == 'V') {
    gps->fix = false;
  } else {
    return false;
  }

  return true;
}

static bool gps_parse_antenna(gps_i2c_esp32_t *gps, char *p) {
  if (gps_is_empty(p)) return false;

  if (p[0] >= '1' && p[0] <= '3') {
    gps->antenna = p[0] - '0';
    return true;
  }

  return false;
}

// The parse function (unchanged)
bool GPS_I2C_ESP32_parse(gps_i2c_esp32_t *gps, char *nmea) {
  if (!gps || !nmea) return false;
  if (!gps_check_nmea(nmea)) return false;

  char *p = nmea + 3;

  if (strncmp(p, "GGA", 3) == 0) {
    p = strchr(p, ',') + 1;
    gps_parse_time(gps, p);
    p = strchr(p, ',') + 1;

    if (gps_parse_coord(p, &gps->latitudeDegrees, &gps->latitude,
                        &gps->latitude_fixed, &gps->lat)) {
      p = strchr(p, ',') + 1;
      p = strchr(p, ',') + 1;
    }

    if (gps_parse_coord(p, &gps->longitudeDegrees, &gps->longitude,
                        &gps->longitude_fixed, &gps->lon)) {
      p = strchr(p, ',') + 1;
      p = strchr(p, ',') + 1;
    }

    if (!gps_is_empty(p)) {
      gps->fixquality = atoi(p);
      if (gps->fixquality > 0) {
        gps->fix = true;
        gps->lastFix = gps->sentTime;
      } else {
        gps->fix = false;
      }
    }
    p = strchr(p, ',') + 1;

    if (!gps_is_empty(p)) gps->satellites = atoi(p);
    p = strchr(p, ',') + 1;

    if (!gps_is_empty(p)) gps->HDOP = atof(p);
    p = strchr(p, ',') + 1;

    if (!gps_is_empty(p)) gps->altitude = atof(p);
    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1;

    if (!gps_is_empty(p)) gps->geoidheight = atof(p);

    gps->lastUpdate = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return true;

  } else if (strncmp(p, "RMC", 3) == 0) {
    p = strchr(p, ',') + 1;
    gps_parse_time(gps, p);
    p = strchr(p, ',') + 1;

    gps_parse_fix(gps, p);
    p = strchr(p, ',') + 1;

    if (gps_parse_coord(p, &gps->latitudeDegrees, &gps->latitude,
                        &gps->latitude_fixed, &gps->lat)) {
      p = strchr(p, ',') + 1;
      p = strchr(p, ',') + 1;
    }

    if (gps_parse_coord(p, &gps->longitudeDegrees, &gps->longitude,
                        &gps->longitude_fixed, &gps->lon)) {
      p = strchr(p, ',') + 1;
      p = strchr(p, ',') + 1;
    }

    if (!gps_is_empty(p)) gps->speed = atof(p);
    p = strchr(p, ',') + 1;

    if (!gps_is_empty(p)) gps->angle = atof(p);
    p = strchr(p, ',') + 1;

    if (!gps_is_empty(p)) {
      uint32_t fulldate = atol(p);
      gps->day = fulldate / 10000;
      gps->month = (fulldate % 10000) / 100;
      gps->year = (fulldate % 100);
      gps->lastDate = gps->sentTime;
    }
    p = strchr(p, ',') + 1;

    if (!gps_is_empty(p)) gps->magvariation = atof(p);
    p = strchr(p, ',') + 1;

    if (!gps_is_empty(p)) gps->mag = *p;

    gps->lastUpdate = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return true;

  } else if (strncmp(p, "GLL", 3) == 0) {
    p = strchr(p, ',') + 1;

    if (gps_parse_coord(p, &gps->latitudeDegrees, &gps->latitude,
                        &gps->latitude_fixed, &gps->lat)) {
      p = strchr(p, ',') + 1;
      p = strchr(p, ',') + 1;
    }

    if (gps_parse_coord(p, &gps->longitudeDegrees, &gps->longitude,
                        &gps->longitude_fixed, &gps->lon)) {
      p = strchr(p, ',') + 1;
      p = strchr(p, ',') + 1;
    }

    gps_parse_time(gps, p);
    p = strchr(p, ',') + 1;
    gps_parse_fix(gps, p);

    gps->lastUpdate = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return true;

  } else if (strncmp(p, "GSA", 3) == 0) {
    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1;

    if (!gps_is_empty(p)) gps->fixquality_3d = atoi(p);
    p = strchr(p, ',') + 1;

    for (int i = 0; i < 12; i++) {
      p = strchr(p, ',') + 1;
    }

    if (!gps_is_empty(p)) gps->PDOP = atof(p);
    p = strchr(p, ',') + 1;

    if (!gps_is_empty(p)) gps->HDOP = atof(p);
    p = strchr(p, ',') + 1;

    if (!gps_is_empty(p)) gps->VDOP = atof(p);

    gps->lastUpdate = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return true;

  } else if (strncmp(p, "TOP", 3) == 0) {
    p = strchr(p, ',') + 1;
    gps_parse_antenna(gps, p);

    gps->lastUpdate = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return true;
  }

  return false;
}

// ============================================================================
// CONTROLE
// ============================================================================

void GPS_I2C_ESP32_pause(gps_i2c_esp32_t *gps, bool p) {
  if (gps) gps->paused = p;
}

bool GPS_I2C_ESP32_standby(gps_i2c_esp32_t *gps) {
  if (!gps) return false;
  if (gps->inStandbyMode) return false;

  gps->inStandbyMode = true;
  GPS_I2C_ESP32_send_command(gps, PMTK_STANDBY);
  return true;
}

bool GPS_I2C_ESP32_wakeup(gps_i2c_esp32_t *gps) {
  if (!gps) return false;
  if (!gps->inStandbyMode) return false;

  gps->inStandbyMode = false;
  // send an empty command or appropriate wake sequence (module dependent)
  GPS_I2C_ESP32_send_command(gps, "");
  return GPS_I2C_ESP32_wait_for_sentence(gps, PMTK_AWAKE, 10, 5000);
}

// ============================================================================
// UTILITAIRES & GETTERS (inchangés)
// ============================================================================

float GPS_I2C_ESP32_seconds_since_fix(gps_i2c_esp32_t *gps) {
  if (!gps) return 0.0f;
  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
  return (now - gps->lastFix) / 1000.0f;
}

float GPS_I2C_ESP32_seconds_since_time(gps_i2c_esp32_t *gps) {
  if (!gps) return 0.0f;
  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
  return (now - gps->lastTime) / 1000.0f;
}

float GPS_I2C_ESP32_seconds_since_date(gps_i2c_esp32_t *gps) {
  if (!gps) return 0.0f;
  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
  return (now - gps->lastDate) / 1000.0f;
}

void GPS_I2C_ESP32_add_checksum(char *buff) {
  if (!buff) return;
  uint8_t cs = 0;
  for (int i = 1; buff[i]; i++) cs ^= buff[i];
  char checksum[5];
  snprintf(checksum, sizeof(checksum), "*%02X", cs);
  strcat(buff, checksum);
}

bool GPS_I2C_ESP32_has_fix(gps_i2c_esp32_t *gps) {
  return gps ? gps->fix : false;
}
uint8_t GPS_I2C_ESP32_get_satellites(gps_i2c_esp32_t *gps) {
  return gps ? gps->satellites : 0;
}
float GPS_I2C_ESP32_get_latitude(gps_i2c_esp32_t *gps) {
  return gps ? gps->latitudeDegrees : 0.0f;
}
float GPS_I2C_ESP32_get_longitude(gps_i2c_esp32_t *gps) {
  return gps ? gps->longitudeDegrees : 0.0f;
}
float GPS_I2C_ESP32_get_altitude(gps_i2c_esp32_t *gps) {
  return gps ? gps->altitude : 0.0f;
}
float GPS_I2C_ESP32_get_speed(gps_i2c_esp32_t *gps) {
  return gps ? gps->speed : 0.0f;
}
float GPS_I2C_ESP32_get_course(gps_i2c_esp32_t *gps) {
  return gps ? gps->angle : 0.0f;
}
float GPS_I2C_ESP32_get_hdop(gps_i2c_esp32_t *gps) {
  return gps ? gps->HDOP : 0.0f;
}
