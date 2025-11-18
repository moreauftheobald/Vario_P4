/**
 * @file config_loader.cpp
 * @brief Implémentation du chargeur de configuration
 * 
 * @author Franck Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#include "config_loader.h"
#include "src/system/sd_manager/sd_manager.h"
#include <LittleFS.h>
#include <cJSON.h>

// Variables externes
extern variometer_config_t g_config;

// Variable privée pour état LittleFS
static bool littlefs_initialized = false;

// Prototypes fonctions privées
static bool init_littlefs();
static bool load_config_from_sd();
static bool load_config_from_littlefs();
static void load_hardcoded_config();
static bool parse_config_json(const char* json_str);
static bool parse_logger_config(cJSON* json);
static bool parse_flight_params_config(cJSON* json);
static bool parse_map_config(cJSON* json);
static bool parse_wifi_config(cJSON* json);
static bool parse_display_config(cJSON* json);
static bool parse_imu_calibration_config(cJSON* json);
static char* serialize_config_to_json();


/**
 * @brief Initialise LittleFS pour accès config flash
 */
static bool init_littlefs() {
  if (littlefs_initialized) {
    return true;  // Déjà initialisé
  }

  if (!LittleFS.begin(false)) {
    Serial.println("[CONFIG] Failed to mount LittleFS");
    return false;
  }

  littlefs_initialized = true;
  Serial.println("[CONFIG] LittleFS mounted");
  return true;
}

/**
 * @brief Charge la configuration selon priorité : SD -> LittleFS -> Hardcodé
 */
bool config_load() {
  Serial.println("[CONFIG] Loading configuration...");

  // Priorité 1 : SD card
  if (sd_manager_is_available()) {
    Serial.println("[CONFIG] Trying SD card...");
    if (load_config_from_sd()) {
      g_config.config_source = CONFIG_SOURCE_SD;
      Serial.println("[CONFIG] Configuration loaded from SD");
      return true;
    }
  }

  // Priorité 2 : LittleFS (flash)
  Serial.println("[CONFIG] Trying LittleFS...");
  if (load_config_from_littlefs()) {
    g_config.config_source = CONFIG_SOURCE_LITTLEFS;
    Serial.println("[CONFIG] Configuration loaded from LittleFS");
    return true;
  }

  // Priorité 3 : Configuration hardcodée
  Serial.println("[CONFIG] Using hardcoded configuration");
  load_hardcoded_config();
  g_config.config_source = CONFIG_SOURCE_HARDCODED;

  return true;
}

/**
 * @brief Charge config depuis SD avec SD manager
 */
static bool load_config_from_sd() {
  char* buffer = NULL;
  size_t size = 0;

  // Utiliser la fonction helper du SD manager (gère lock/unlock)
  if (!sd_manager_read_file(CONFIG_PATH_SD, &buffer, &size)) {
    Serial.println("[CONFIG] Failed to read config from SD");
    return false;
  }

  // Parser le JSON
  bool success = parse_config_json(buffer);

  // Libérer le buffer
  free(buffer);

  return success;
}

/**
 * @brief Sérialise g_config en JSON
 * @return Pointeur vers string JSON (à libérer avec free()) ou NULL si erreur
 */
static char* serialize_config_to_json() {
  cJSON* root = cJSON_CreateObject();

  // Section logger
  cJSON* logger = cJSON_CreateObject();
  cJSON_AddStringToObject(logger, "output", g_config.logger.output);
  cJSON_AddStringToObject(logger, "kalman", g_config.logger.kalman);
  cJSON_AddStringToObject(logger, "i2c", g_config.logger.i2c);
  cJSON_AddStringToObject(logger, "bmp585", g_config.logger.bmp585);
  cJSON_AddStringToObject(logger, "imu", g_config.logger.imu);
  cJSON_AddStringToObject(logger, "gps", g_config.logger.gps);
  cJSON_AddStringToObject(logger, "theme", g_config.logger.theme);
  cJSON_AddStringToObject(logger, "display", g_config.logger.display);
  cJSON_AddStringToObject(logger, "map", g_config.logger.map);
  cJSON_AddStringToObject(logger, "wifi", g_config.logger.wifi);
  cJSON_AddStringToObject(logger, "storage", g_config.logger.storage);
  cJSON_AddStringToObject(logger, "flight", g_config.logger.flight);
  cJSON_AddStringToObject(logger, "system", g_config.logger.system);
  cJSON_AddStringToObject(logger, "memory", g_config.logger.memory);
  cJSON_AddItemToObject(root, "logger", logger);

  // Section flight_params
  cJSON* flight = cJSON_CreateObject();
  cJSON_AddNumberToObject(flight, "vario_damping", g_config.flight_params.vario_damping);
  cJSON_AddNumberToObject(flight, "vario_integration_time", g_config.flight_params.vario_integration_time);
  cJSON_AddNumberToObject(flight, "qnh", g_config.flight_params.qnh);
  cJSON_AddNumberToObject(flight, "vario_threshold_strong", g_config.flight_params.vario_threshold_strong);
  cJSON_AddNumberToObject(flight, "vario_threshold_medium", g_config.flight_params.vario_threshold_medium);
  cJSON_AddNumberToObject(flight, "vario_threshold_weak", g_config.flight_params.vario_threshold_weak);
  cJSON_AddItemToObject(root, "flight_params", flight);

  // Section map
  cJSON* map = cJSON_CreateObject();
  cJSON_AddNumberToObject(map, "zoom_default", g_config.map.zoom_default);
  cJSON_AddNumberToObject(map, "zoom_min", g_config.map.zoom_min);
  cJSON_AddNumberToObject(map, "zoom_max", g_config.map.zoom_max);
  cJSON_AddNumberToObject(map, "tile_max_age_days", g_config.map.tile_max_age_days);
  cJSON_AddNumberToObject(map, "cache_size_mb", g_config.map.cache_size_mb);
  cJSON_AddItemToObject(root, "map", map);

  // Section wifi
  cJSON* wifi = cJSON_CreateObject();
  cJSON_AddStringToObject(wifi, "ssid", g_config.wifi.ssid);
  cJSON_AddStringToObject(wifi, "password", g_config.wifi.password);
  cJSON_AddBoolToObject(wifi, "auto_connect", g_config.wifi.auto_connect);
  cJSON_AddItemToObject(root, "wifi", wifi);

  // Section display
  cJSON* display = cJSON_CreateObject();
  cJSON_AddNumberToObject(display, "brightness", g_config.display.brightness);
  cJSON_AddNumberToObject(display, "refresh_rate", g_config.display.refresh_rate);
  cJSON_AddBoolToObject(display, "auto_brightness", g_config.display.auto_brightness);
  cJSON_AddItemToObject(root, "display", display);

  // Section imu_calibration
  cJSON* imu_cal = cJSON_CreateObject();

  cJSON* gyro = cJSON_CreateObject();
  cJSON_AddNumberToObject(gyro, "x", g_config.imu_calibration.gyro_offset.x);
  cJSON_AddNumberToObject(gyro, "y", g_config.imu_calibration.gyro_offset.y);
  cJSON_AddNumberToObject(gyro, "z", g_config.imu_calibration.gyro_offset.z);
  cJSON_AddItemToObject(imu_cal, "gyro_offset", gyro);

  cJSON* accel = cJSON_CreateObject();
  cJSON_AddNumberToObject(accel, "x", g_config.imu_calibration.accel_offset.x);
  cJSON_AddNumberToObject(accel, "y", g_config.imu_calibration.accel_offset.y);
  cJSON_AddNumberToObject(accel, "z", g_config.imu_calibration.accel_offset.z);
  cJSON_AddItemToObject(imu_cal, "accel_offset", accel);

  cJSON* scale = cJSON_CreateObject();
  cJSON_AddNumberToObject(scale, "x", g_config.imu_calibration.accel_scale.x);
  cJSON_AddNumberToObject(scale, "y", g_config.imu_calibration.accel_scale.y);
  cJSON_AddNumberToObject(scale, "z", g_config.imu_calibration.accel_scale.z);
  cJSON_AddItemToObject(imu_cal, "accel_scale", scale);

  cJSON* metadata = cJSON_CreateObject();
  cJSON_AddNumberToObject(metadata, "timestamp", g_config.imu_calibration.metadata.timestamp);
  cJSON_AddNumberToObject(metadata, "temperature", g_config.imu_calibration.metadata.temperature);
  cJSON_AddNumberToObject(metadata, "quality_score", g_config.imu_calibration.metadata.quality_score);
  cJSON_AddStringToObject(metadata, "location", g_config.imu_calibration.metadata.location);
  cJSON_AddItemToObject(imu_cal, "metadata", metadata);

  cJSON_AddItemToObject(root, "imu_calibration", imu_cal);

  // Convertir en string
  char* json_str = cJSON_Print(root);
  cJSON_Delete(root);

  if (!json_str) {
    Serial.println("[CONFIG] Failed to serialize JSON");
    return NULL;
  }

  return json_str;
}

/**
 * @brief Sauvegarde config sur LittleFS
 */
bool config_save_to_littlefs() {
  if (!init_littlefs()) {
    Serial.println("[CONFIG] LittleFS not available for saving");
    return false;
  }

  char* json_str = serialize_config_to_json();
  if (!json_str) {
    return false;
  }

  File file = LittleFS.open(CONFIG_PATH_LITTLEFS, "w");
  if (!file) {
    Serial.println("[CONFIG] Failed to open LittleFS for writing");
    free(json_str);
    return false;
  }

  size_t written = file.print(json_str);
  file.close();
  free(json_str);

  if (written > 0) {
    Serial.println("[CONFIG] Configuration saved to LittleFS");
    return true;
  } else {
    Serial.println("[CONFIG] Failed to write to LittleFS");
    return false;
  }
}

/**
 * @brief Charge config depuis LittleFS
 */
static bool load_config_from_littlefs() {
  // Initialiser LittleFS si pas déjà fait
  if (!init_littlefs()) {
    return false;
  }

  if (!LittleFS.exists(CONFIG_PATH_LITTLEFS)) {
    Serial.println("[CONFIG] Config file not found in LittleFS");
    return false;
  }

  File file = LittleFS.open(CONFIG_PATH_LITTLEFS, "r");
  if (!file) {
    Serial.println("[CONFIG] Failed to open config from LittleFS");
    return false;
  }

  size_t size = file.size();
  char* buffer = (char*)malloc(size + 1);
  if (!buffer) {
    Serial.println("[CONFIG] Memory allocation failed");
    file.close();
    return false;
  }

  file.read((uint8_t*)buffer, size);
  buffer[size] = '\0';
  file.close();

  bool success = parse_config_json(buffer);
  free(buffer);

  return success;
}

/**
 * @brief Charge la configuration par défaut
 */
static void load_hardcoded_config() {
  // Logger par défaut
  strncpy(g_config.logger.output, "UART", CONFIG_LOG_LEVEL_MAX);
  strncpy(g_config.logger.kalman, "Info", CONFIG_LOG_LEVEL_MAX);
  strncpy(g_config.logger.i2c, "None", CONFIG_LOG_LEVEL_MAX);
  strncpy(g_config.logger.bmp585, "Error", CONFIG_LOG_LEVEL_MAX);
  strncpy(g_config.logger.imu, "Info", CONFIG_LOG_LEVEL_MAX);
  strncpy(g_config.logger.gps, "Verbose", CONFIG_LOG_LEVEL_MAX);
  strncpy(g_config.logger.theme, "Warning", CONFIG_LOG_LEVEL_MAX);
  strncpy(g_config.logger.display, "Info", CONFIG_LOG_LEVEL_MAX);
  strncpy(g_config.logger.map, "Verbose", CONFIG_LOG_LEVEL_MAX);
  strncpy(g_config.logger.wifi, "Info", CONFIG_LOG_LEVEL_MAX);
  strncpy(g_config.logger.storage, "Error", CONFIG_LOG_LEVEL_MAX);
  strncpy(g_config.logger.flight, "Verbose", CONFIG_LOG_LEVEL_MAX);
  strncpy(g_config.logger.system, "Info", CONFIG_LOG_LEVEL_MAX);
  strncpy(g_config.logger.memory, "Info", CONFIG_LOG_LEVEL_MAX);

  // Paramètres de vol par défaut
  g_config.flight_params.vario_damping = 0.8f;
  g_config.flight_params.vario_integration_time = 2.0f;
  g_config.flight_params.qnh = 1013.25f;
  g_config.flight_params.vario_threshold_strong = 3.0f;
  g_config.flight_params.vario_threshold_medium = 1.5f;
  g_config.flight_params.vario_threshold_weak = 0.5f;

  // Carte par défaut
  g_config.map.zoom_default = 14;
  g_config.map.zoom_min = 10;
  g_config.map.zoom_max = 18;
  g_config.map.tile_max_age_days = 180;
  g_config.map.cache_size_mb = 100;

  // WiFi par défaut
  strncpy(g_config.wifi.ssid, "", CONFIG_STRING_MAX);
  strncpy(g_config.wifi.password, "", CONFIG_STRING_MAX);
  g_config.wifi.auto_connect = false;

  // Display par défaut
  g_config.display.brightness = 80;
  g_config.display.refresh_rate = 30;
  g_config.display.auto_brightness = true;

  Serial.println("[CONFIG] Hardcoded defaults loaded");
}

/**
 * @brief Parse le JSON de configuration
 */
static bool parse_config_json(const char* json_str) {
  cJSON* root = cJSON_Parse(json_str);
  if (root == NULL) {
    const char* error_ptr = cJSON_GetErrorPtr();
    if (error_ptr != NULL) {
      Serial.printf("[CONFIG] JSON error: %s\n", error_ptr);
    }
    return false;
  }

  // Parser chaque section
  bool success = true;

  success &= parse_logger_config(root);
  success &= parse_flight_params_config(root);
  success &= parse_map_config(root);
  success &= parse_wifi_config(root);
  success &= parse_display_config(root);
  success &= parse_imu_calibration_config(root);  // AJOUT ICI

  cJSON_Delete(root);

  if (!config_validate(&g_config)) {
    Serial.println("[CONFIG] Config validation failed");
    return false;
  }

  Serial.println("[CONFIG] Configuration parsed successfully");
  return success;
}

/**
 * @brief Parse la section logger
 */
static bool parse_logger_config(cJSON* json) {
  cJSON* logger = cJSON_GetObjectItem(json, "logger");
  if (!logger) {
    Serial.println("[CONFIG] Missing logger section");
    return false;
  }

  Serial.println("[CONFIG] Parsing logger config...");

// Helper macro pour parser les champs
#define PARSE_LOG_FIELD(field) \
  do { \
    cJSON* item = cJSON_GetObjectItem(logger, #field); \
    if (item && item->valuestring) { \
      strncpy(g_config.logger.field, item->valuestring, CONFIG_LOG_LEVEL_MAX - 1); \
      Serial.printf("[CONFIG]   %s = %s\n", #field, g_config.logger.field); \
    } \
  } while (0)

  PARSE_LOG_FIELD(output);
  PARSE_LOG_FIELD(kalman);
  PARSE_LOG_FIELD(i2c);
  PARSE_LOG_FIELD(bmp585);
  PARSE_LOG_FIELD(imu);
  PARSE_LOG_FIELD(gps);
  PARSE_LOG_FIELD(theme);
  PARSE_LOG_FIELD(display);
  PARSE_LOG_FIELD(map);
  PARSE_LOG_FIELD(wifi);
  PARSE_LOG_FIELD(storage);
  PARSE_LOG_FIELD(flight);
  PARSE_LOG_FIELD(system);
  PARSE_LOG_FIELD(memory);

#undef PARSE_LOG_FIELD

  return true;
}

/**
 * @brief Parse la section flight_params
 */
static bool parse_flight_params_config(cJSON* json) {
  cJSON* flight = cJSON_GetObjectItem(json, "flight_params");
  if (!flight) {
    Serial.println("[CONFIG] Missing flight_params section");
    return false;
  }

  Serial.println("[CONFIG] Parsing flight_params config...");

#define PARSE_FLOAT_FIELD(field) \
  do { \
    cJSON* item = cJSON_GetObjectItem(flight, #field); \
    if (item && cJSON_IsNumber(item)) { \
      g_config.flight_params.field = (float)item->valuedouble; \
      Serial.printf("[CONFIG]   %s = %.2f\n", #field, g_config.flight_params.field); \
    } \
  } while (0)

  PARSE_FLOAT_FIELD(vario_damping);
  PARSE_FLOAT_FIELD(vario_integration_time);
  PARSE_FLOAT_FIELD(qnh);
  PARSE_FLOAT_FIELD(vario_threshold_strong);
  PARSE_FLOAT_FIELD(vario_threshold_medium);
  PARSE_FLOAT_FIELD(vario_threshold_weak);

#undef PARSE_FLOAT_FIELD

  return true;
}

/**
 * @brief Parse la section map
 */
static bool parse_map_config(cJSON* json) {
  cJSON* map = cJSON_GetObjectItem(json, "map");
  if (!map) {
    Serial.println("[CONFIG] Missing map section");
    return false;
  }

  Serial.println("[CONFIG] Parsing map config...");

#define PARSE_UINT_FIELD(field) \
  do { \
    cJSON* item = cJSON_GetObjectItem(map, #field); \
    if (item && cJSON_IsNumber(item)) { \
      g_config.map.field = (uint16_t)item->valueint; \
      Serial.printf("[CONFIG]   %s = %d\n", #field, g_config.map.field); \
    } \
  } while (0)

  PARSE_UINT_FIELD(zoom_default);
  PARSE_UINT_FIELD(zoom_min);
  PARSE_UINT_FIELD(zoom_max);
  PARSE_UINT_FIELD(tile_max_age_days);
  PARSE_UINT_FIELD(cache_size_mb);

#undef PARSE_UINT_FIELD

  return true;
}

/**
 * @brief Parse la section wifi
 */
static bool parse_wifi_config(cJSON* json) {
  cJSON* wifi = cJSON_GetObjectItem(json, "wifi");
  if (!wifi) {
    Serial.println("[CONFIG] Missing wifi section");
    return false;
  }

  Serial.println("[CONFIG] Parsing wifi config...");

  cJSON* ssid = cJSON_GetObjectItem(wifi, "ssid");
  if (ssid && ssid->valuestring) {
    strncpy(g_config.wifi.ssid, ssid->valuestring, CONFIG_STRING_MAX - 1);
    Serial.printf("[CONFIG]   ssid = %s\n", g_config.wifi.ssid);
  }

  cJSON* password = cJSON_GetObjectItem(wifi, "password");
  if (password && password->valuestring) {
    strncpy(g_config.wifi.password, password->valuestring, CONFIG_STRING_MAX - 1);
    Serial.println("[CONFIG]   password = ***");
  }

  cJSON* auto_connect = cJSON_GetObjectItem(wifi, "auto_connect");
  if (auto_connect && cJSON_IsBool(auto_connect)) {
    g_config.wifi.auto_connect = cJSON_IsTrue(auto_connect);
    Serial.printf("[CONFIG]   auto_connect = %s\n",
                  g_config.wifi.auto_connect ? "true" : "false");
  }

  return true;
}

/**
 * @brief Parse la section display
 */
static bool parse_display_config(cJSON* json) {
  cJSON* display = cJSON_GetObjectItem(json, "display");
  if (!display) {
    Serial.println("[CONFIG] Missing display section");
    return false;
  }

  Serial.println("[CONFIG] Parsing display config...");

  cJSON* brightness = cJSON_GetObjectItem(display, "brightness");
  if (brightness && cJSON_IsNumber(brightness)) {
    g_config.display.brightness = (uint8_t)brightness->valueint;
    Serial.printf("[CONFIG]   brightness = %d\n", g_config.display.brightness);
  }

  cJSON* refresh_rate = cJSON_GetObjectItem(display, "refresh_rate");
  if (refresh_rate && cJSON_IsNumber(refresh_rate)) {
    g_config.display.refresh_rate = (uint16_t)refresh_rate->valueint;
    Serial.printf("[CONFIG]   refresh_rate = %d\n", g_config.display.refresh_rate);
  }

  cJSON* auto_brightness = cJSON_GetObjectItem(display, "auto_brightness");
  if (auto_brightness && cJSON_IsBool(auto_brightness)) {
    g_config.display.auto_brightness = cJSON_IsTrue(auto_brightness);
    Serial.printf("[CONFIG]   auto_brightness = %s\n",
                  g_config.display.auto_brightness ? "true" : "false");
  }

  return true;
}

/**
 * @brief Parse la section imu_calibration
 */
static bool parse_imu_calibration_config(cJSON* json) {
  cJSON* imu_cal = cJSON_GetObjectItem(json, "imu_calibration");
  if (!imu_cal) {
    Serial.println("[CONFIG] No IMU calibration in config");
    return true;  // Pas une erreur
  }

  Serial.println("[CONFIG] Parsing IMU calibration...");

  // Gyro offset
  cJSON* gyro = cJSON_GetObjectItem(imu_cal, "gyro_offset");
  if (gyro) {
    cJSON* x = cJSON_GetObjectItem(gyro, "x");
    cJSON* y = cJSON_GetObjectItem(gyro, "y");
    cJSON* z = cJSON_GetObjectItem(gyro, "z");
    if (x) g_config.imu_calibration.gyro_offset.x = (float)x->valuedouble;
    if (y) g_config.imu_calibration.gyro_offset.y = (float)y->valuedouble;
    if (z) g_config.imu_calibration.gyro_offset.z = (float)z->valuedouble;
  }

  // Accel offset
  cJSON* accel = cJSON_GetObjectItem(imu_cal, "accel_offset");
  if (accel) {
    cJSON* x = cJSON_GetObjectItem(accel, "x");
    cJSON* y = cJSON_GetObjectItem(accel, "y");
    cJSON* z = cJSON_GetObjectItem(accel, "z");
    if (x) g_config.imu_calibration.accel_offset.x = (float)x->valuedouble;
    if (y) g_config.imu_calibration.accel_offset.y = (float)y->valuedouble;
    if (z) g_config.imu_calibration.accel_offset.z = (float)z->valuedouble;
  }

  // Accel scale
  cJSON* scale = cJSON_GetObjectItem(imu_cal, "accel_scale");
  if (scale) {
    cJSON* x = cJSON_GetObjectItem(scale, "x");
    cJSON* y = cJSON_GetObjectItem(scale, "y");
    cJSON* z = cJSON_GetObjectItem(scale, "z");
    if (x) g_config.imu_calibration.accel_scale.x = (float)x->valuedouble;
    if (y) g_config.imu_calibration.accel_scale.y = (float)y->valuedouble;
    if (z) g_config.imu_calibration.accel_scale.z = (float)z->valuedouble;
  }

  // Metadata
  cJSON* metadata = cJSON_GetObjectItem(imu_cal, "metadata");
  if (metadata) {
    cJSON* ts = cJSON_GetObjectItem(metadata, "timestamp");
    cJSON* temp = cJSON_GetObjectItem(metadata, "temperature");
    cJSON* score = cJSON_GetObjectItem(metadata, "quality_score");
    cJSON* loc = cJSON_GetObjectItem(metadata, "location");

    if (ts) g_config.imu_calibration.metadata.timestamp = (uint32_t)ts->valueint;
    if (temp) g_config.imu_calibration.metadata.temperature = (float)temp->valuedouble;
    if (score) g_config.imu_calibration.metadata.quality_score = (uint8_t)score->valueint;
    if (loc && loc->valuestring) {
      strncpy(g_config.imu_calibration.metadata.location,
              loc->valuestring, 15);
      g_config.imu_calibration.metadata.location[15] = '\0';
    }
  }

  Serial.println("[CONFIG] IMU calibration loaded from config");
  return true;
}

/**
 * @brief Valide la configuration chargée
 */
bool config_validate(variometer_config_t* config) {
  bool valid = true;

  // Valider flight_params
  if (config->flight_params.vario_damping < 0.0f || config->flight_params.vario_damping > 1.0f) {
    Serial.println("[CONFIG] Invalid vario_damping (must be 0.0-1.0)");
    config->flight_params.vario_damping = 0.8f;
    valid = false;
  }

  if (config->flight_params.qnh < 900.0f || config->flight_params.qnh > 1100.0f) {
    Serial.println("[CONFIG] Invalid QNH (must be 900-1100 hPa)");
    config->flight_params.qnh = 1013.25f;
    valid = false;
  }

  // Valider map
  if (config->map.zoom_default < config->map.zoom_min || config->map.zoom_default > config->map.zoom_max) {
    Serial.println("[CONFIG] Invalid zoom_default");
    config->map.zoom_default = 14;
    valid = false;
  }

  // Valider display
  if (config->display.brightness > 100) {
    Serial.println("[CONFIG] Invalid brightness (must be 0-100)");
    config->display.brightness = 80;
    valid = false;
  }

  return valid;
}

/**
 * @brief Sauvegarde config sur SD
 */
bool config_save() {
  char* json_str = serialize_config_to_json();
  if (!json_str) {
    return false;
  }

  bool success = sd_manager_write_file(CONFIG_PATH_SD, json_str, strlen(json_str));

  free(json_str);

  if (success) {
    Serial.println("[CONFIG] Configuration saved to SD");
  } else {
    Serial.println("[CONFIG] Failed to save configuration");
  }

  return success;
}