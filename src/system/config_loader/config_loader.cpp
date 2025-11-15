/**
 * @file config_loader.cpp
 * @brief Implementation du chargeur de configuration
 * 
 * @author Theobald Moreau
 * @date 2025-11-14
 * @version 1.0
 */

#include "config_loader.h"
#include <string.h>

// Variable globale de configuration
variometer_config_t g_config = {0};

// Forward declarations pour fonctions privees
static bool parse_logger_config(cJSON* json);
static bool parse_flight_params_config(cJSON* json);
static bool parse_map_config(cJSON* json);
static bool parse_wifi_config(cJSON* json);
static bool parse_display_config(cJSON* json);
static void set_default_config();

/**
 * @brief Initialise le systeme de configuration
 */
bool config_init() {
    Serial.println("[CONFIG] Initializing configuration system...");
    
    // Priorite 1: Tenter de charger depuis SD_MMC
    if (config_load_from_sd()) {
        g_config.config_source = CONFIG_SOURCE_SD;
        Serial.println("[CONFIG] Config loaded from SD_MMC");
        return true;
    }
    
    // Priorite 2: Tenter de charger depuis SPIFFS
    Serial.println("[CONFIG] SD config not found, trying SPIFFS...");
    if (config_load_from_spiffs()) {
        g_config.config_source = CONFIG_SOURCE_SPIFFS;
        Serial.println("[CONFIG] Config loaded from SPIFFS");
        return true;
    }
    
    // Priorite 3: Fallback sur configuration par defaut hardcodee
    Serial.println("[CONFIG] SPIFFS config not found, using hardcoded default");
    if (config_load_from_flash()) {
        g_config.config_source = CONFIG_SOURCE_HARDCODED;
        Serial.println("[CONFIG] Config loaded from flash (hardcoded default)");
        return true;
    }
    
    // En dernier recours, config hardcodee directe
    Serial.println("[CONFIG] Failed to load any config, using minimal hardcoded defaults");
    set_default_config();
    g_config.config_source = CONFIG_SOURCE_HARDCODED;
    return false;
}

/**
 * @brief Charge la configuration depuis SD_MMC
 */
bool config_load_from_sd() {
    if (!SD_MMC.exists(CONFIG_FILE_PATH_SD)) {
        return false;
    }
    
    File file = SD_MMC.open(CONFIG_FILE_PATH_SD, FILE_READ);
    if (!file) {
        Serial.println("[CONFIG] Failed to open config file from SD");
        return false;
    }
    
    // Lire le contenu du fichier
    String json_content = file.readString();
    file.close();
    
    if (json_content.length() == 0) {
        Serial.println("[CONFIG] Config file on SD is empty");
        return false;
    }
    
    // Parser le JSON
    return config_parse_json(json_content.c_str());
}

/**
 * @brief Charge la configuration depuis SPIFFS
 */
bool config_load_from_spiffs() {
    // SPIFFS doit etre initialise avant
    if (!SPIFFS.begin(false)) {  // false = ne pas formater si echec
        Serial.println("[CONFIG] Failed to mount SPIFFS");
        return false;
    }
    
    if (!SPIFFS.exists(CONFIG_FILE_PATH_SPIFFS)) {
        Serial.println("[CONFIG] Config file not found in SPIFFS");
        return false;
    }
    
    File file = SPIFFS.open(CONFIG_FILE_PATH_SPIFFS, FILE_READ);
    if (!file) {
        Serial.println("[CONFIG] Failed to open config file from SPIFFS");
        return false;
    }
    
    // Lire le contenu du fichier
    String json_content = file.readString();
    file.close();
    
    if (json_content.length() == 0) {
        Serial.println("[CONFIG] Config file in SPIFFS is empty");
        return false;
    }
    
    Serial.println("[CONFIG] Parsing config from SPIFFS...");
    
    // Parser le JSON
    return config_parse_json(json_content.c_str());
}

/**
 * @brief Charge la configuration par defaut depuis flash
 */
bool config_load_from_flash() {
    // Copier depuis PROGMEM
    char json_buffer[2048];
    strcpy_P(json_buffer, DEFAULT_CONFIG_JSON);
    
    return config_parse_json(json_buffer);
}

/**
 * @brief Parse le JSON et remplit la structure config
 */
bool config_parse_json(const char* json_string) {
    Serial.println("[CONFIG] Parsing JSON configuration...");
    
    cJSON* root = cJSON_Parse(json_string);
    if (root == NULL) {
        Serial.println("[CONFIG] Failed to parse config JSON");
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
    
    cJSON_Delete(root);
    
    // Valider la configuration
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
        } while(0)
    
    PARSE_LOG_FIELD(output);
    PARSE_LOG_FIELD(kalman);
    PARSE_LOG_FIELD(i2c);
    PARSE_LOG_FIELD(bmp390);
    PARSE_LOG_FIELD(imu);
    PARSE_LOG_FIELD(gps);
    PARSE_LOG_FIELD(theme);
    PARSE_LOG_FIELD(display);
    PARSE_LOG_FIELD(map);
    PARSE_LOG_FIELD(wifi);
    PARSE_LOG_FIELD(storage);
    PARSE_LOG_FIELD(flight);
    PARSE_LOG_FIELD(system);
    
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
    
    cJSON* item;
    
    item = cJSON_GetObjectItem(flight, "vario_damping");
    if (item) {
        g_config.flight_params.vario_damping = (float)item->valuedouble;
        Serial.printf("[CONFIG]   vario_damping = %.2f\n", g_config.flight_params.vario_damping);
    }
    
    item = cJSON_GetObjectItem(flight, "vario_integration_time");
    if (item) {
        g_config.flight_params.vario_integration_time = (float)item->valuedouble;
        Serial.printf("[CONFIG]   vario_integration_time = %.2f\n", g_config.flight_params.vario_integration_time);
    }
    
    item = cJSON_GetObjectItem(flight, "qnh");
    if (item) {
        g_config.flight_params.qnh = (float)item->valuedouble;
        Serial.printf("[CONFIG]   qnh = %.2f\n", g_config.flight_params.qnh);
    }
    
    item = cJSON_GetObjectItem(flight, "vario_threshold_strong");
    if (item) {
        g_config.flight_params.vario_threshold_strong = (float)item->valuedouble;
        Serial.printf("[CONFIG]   vario_threshold_strong = %.2f\n", g_config.flight_params.vario_threshold_strong);
    }
    
    item = cJSON_GetObjectItem(flight, "vario_threshold_medium");
    if (item) {
        g_config.flight_params.vario_threshold_medium = (float)item->valuedouble;
        Serial.printf("[CONFIG]   vario_threshold_medium = %.2f\n", g_config.flight_params.vario_threshold_medium);
    }
    
    item = cJSON_GetObjectItem(flight, "vario_threshold_weak");
    if (item) {
        g_config.flight_params.vario_threshold_weak = (float)item->valuedouble;
        Serial.printf("[CONFIG]   vario_threshold_weak = %.2f\n", g_config.flight_params.vario_threshold_weak);
    }
    
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
    
    cJSON* item;
    
    item = cJSON_GetObjectItem(map, "zoom_default");
    if (item) {
        g_config.map.zoom_default = (uint8_t)item->valueint;
        Serial.printf("[CONFIG]   zoom_default = %d\n", g_config.map.zoom_default);
    }
    
    item = cJSON_GetObjectItem(map, "zoom_min");
    if (item) {
        g_config.map.zoom_min = (uint8_t)item->valueint;
        Serial.printf("[CONFIG]   zoom_min = %d\n", g_config.map.zoom_min);
    }
    
    item = cJSON_GetObjectItem(map, "zoom_max");
    if (item) {
        g_config.map.zoom_max = (uint8_t)item->valueint;
        Serial.printf("[CONFIG]   zoom_max = %d\n", g_config.map.zoom_max);
    }
    
    item = cJSON_GetObjectItem(map, "tile_max_age_days");
    if (item) {
        g_config.map.tile_max_age_days = (uint16_t)item->valueint;
        Serial.printf("[CONFIG]   tile_max_age_days = %d\n", g_config.map.tile_max_age_days);
    }
    
    item = cJSON_GetObjectItem(map, "cache_size_mb");
    if (item) {
        g_config.map.cache_size_mb = (uint16_t)item->valueint;
        Serial.printf("[CONFIG]   cache_size_mb = %d\n", g_config.map.cache_size_mb);
    }
    
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
    
    cJSON* item;
    
    item = cJSON_GetObjectItem(wifi, "ssid");
    if (item && item->valuestring) {
        strncpy(g_config.wifi.ssid, item->valuestring, CONFIG_STRING_MAX - 1);
        Serial.printf("[CONFIG]   ssid = %s\n", g_config.wifi.ssid);
    }
    
    item = cJSON_GetObjectItem(wifi, "password");
    if (item && item->valuestring) {
        strncpy(g_config.wifi.password, item->valuestring, CONFIG_STRING_MAX - 1);
        Serial.println("[CONFIG]   password = [HIDDEN]");
    }
    
    item = cJSON_GetObjectItem(wifi, "auto_connect");
    if (item) {
        g_config.wifi.auto_connect = cJSON_IsTrue(item);
        Serial.printf("[CONFIG]   auto_connect = %s\n", g_config.wifi.auto_connect ? "true" : "false");
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
    
    cJSON* item;
    
    item = cJSON_GetObjectItem(display, "brightness");
    if (item) {
        g_config.display.brightness = (uint8_t)item->valueint;
        Serial.printf("[CONFIG]   brightness = %d\n", g_config.display.brightness);
    }
    
    item = cJSON_GetObjectItem(display, "refresh_rate");
    if (item) {
        g_config.display.refresh_rate = (uint16_t)item->valueint;
        Serial.printf("[CONFIG]   refresh_rate = %d\n", g_config.display.refresh_rate);
    }
    
    item = cJSON_GetObjectItem(display, "auto_brightness");
    if (item) {
        g_config.display.auto_brightness = cJSON_IsTrue(item);
        Serial.printf("[CONFIG]   auto_brightness = %s\n", g_config.display.auto_brightness ? "true" : "false");
    }
    
    return true;
}

/**
 * @brief Valide la configuration
 */
bool config_validate(variometer_config_t* config) {
    bool valid = true;
    
    // Valider flight_params
    if (config->flight_params.vario_damping < 0.0f || config->flight_params.vario_damping > 1.0f) {
        Serial.printf("[CONFIG] Invalid vario_damping: %.2f, setting to 0.5\n", config->flight_params.vario_damping);
        config->flight_params.vario_damping = 0.5f;
        valid = false;
    }
    
    if (config->flight_params.qnh < 900.0f || config->flight_params.qnh > 1100.0f) {
        Serial.printf("[CONFIG] Invalid QNH: %.2f, setting to 1013.25\n", config->flight_params.qnh);
        config->flight_params.qnh = 1013.25f;
        valid = false;
    }
    
    // Valider map
    if (config->map.zoom_min > config->map.zoom_max) {
        Serial.printf("[CONFIG] Invalid zoom range: %d-%d, correcting\n", config->map.zoom_min, config->map.zoom_max);
        config->map.zoom_min = 10;
        config->map.zoom_max = 18;
        valid = false;
    }
    
    if (config->map.zoom_default < config->map.zoom_min || config->map.zoom_default > config->map.zoom_max) {
        Serial.printf("[CONFIG] Invalid zoom_default: %d, setting to %d\n", 
                     config->map.zoom_default, (config->map.zoom_min + config->map.zoom_max) / 2);
        config->map.zoom_default = (config->map.zoom_min + config->map.zoom_max) / 2;
        valid = false;
    }
    
    // Valider display
    if (config->display.brightness > 100) {
        Serial.printf("[CONFIG] Invalid brightness: %d, setting to 80\n", config->display.brightness);
        config->display.brightness = 80;
        valid = false;
    }
    
    return valid;
}

/**
 * @brief Sauvegarde la configuration sur SD
 */
bool config_save_to_sd() {
    Serial.println("[CONFIG] Saving configuration to SD...");
    
    // Creer l'objet JSON
    cJSON* root = cJSON_CreateObject();
    
    // Logger
    cJSON* logger = cJSON_CreateObject();
    cJSON_AddStringToObject(logger, "output", g_config.logger.output);
    cJSON_AddStringToObject(logger, "kalman", g_config.logger.kalman);
    cJSON_AddStringToObject(logger, "i2c", g_config.logger.i2c);
    cJSON_AddStringToObject(logger, "bmp390", g_config.logger.bmp390);
    cJSON_AddStringToObject(logger, "imu", g_config.logger.imu);
    cJSON_AddStringToObject(logger, "gps", g_config.logger.gps);
    cJSON_AddStringToObject(logger, "theme", g_config.logger.theme);
    cJSON_AddStringToObject(logger, "display", g_config.logger.display);
    cJSON_AddStringToObject(logger, "map", g_config.logger.map);
    cJSON_AddStringToObject(logger, "wifi", g_config.logger.wifi);
    cJSON_AddStringToObject(logger, "storage", g_config.logger.storage);
    cJSON_AddStringToObject(logger, "flight", g_config.logger.flight);
    cJSON_AddStringToObject(logger, "system", g_config.logger.system);
    cJSON_AddItemToObject(root, "logger", logger);
    
    // Flight params
    cJSON* flight = cJSON_CreateObject();
    cJSON_AddNumberToObject(flight, "vario_damping", g_config.flight_params.vario_damping);
    cJSON_AddNumberToObject(flight, "vario_integration_time", g_config.flight_params.vario_integration_time);
    cJSON_AddNumberToObject(flight, "qnh", g_config.flight_params.qnh);
    cJSON_AddNumberToObject(flight, "vario_threshold_strong", g_config.flight_params.vario_threshold_strong);
    cJSON_AddNumberToObject(flight, "vario_threshold_medium", g_config.flight_params.vario_threshold_medium);
    cJSON_AddNumberToObject(flight, "vario_threshold_weak", g_config.flight_params.vario_threshold_weak);
    cJSON_AddItemToObject(root, "flight_params", flight);
    
    // Map
    cJSON* map = cJSON_CreateObject();
    cJSON_AddNumberToObject(map, "zoom_default", g_config.map.zoom_default);
    cJSON_AddNumberToObject(map, "zoom_min", g_config.map.zoom_min);
    cJSON_AddNumberToObject(map, "zoom_max", g_config.map.zoom_max);
    cJSON_AddNumberToObject(map, "tile_max_age_days", g_config.map.tile_max_age_days);
    cJSON_AddNumberToObject(map, "cache_size_mb", g_config.map.cache_size_mb);
    cJSON_AddItemToObject(root, "map", map);
    
    // WiFi
    cJSON* wifi = cJSON_CreateObject();
    cJSON_AddStringToObject(wifi, "ssid", g_config.wifi.ssid);
    cJSON_AddStringToObject(wifi, "password", g_config.wifi.password);
    cJSON_AddBoolToObject(wifi, "auto_connect", g_config.wifi.auto_connect);
    cJSON_AddItemToObject(root, "wifi", wifi);
    
    // Display
    cJSON* display = cJSON_CreateObject();
    cJSON_AddNumberToObject(display, "brightness", g_config.display.brightness);
    cJSON_AddNumberToObject(display, "refresh_rate", g_config.display.refresh_rate);
    cJSON_AddBoolToObject(display, "auto_brightness", g_config.display.auto_brightness);
    cJSON_AddItemToObject(root, "display", display);
    
    // Convertir en string
    char* json_string = cJSON_Print(root);
    cJSON_Delete(root);
    
    if (!json_string) {
        Serial.println("[CONFIG] Failed to create JSON string");
        return false;
    }
    
    // Ecrire sur SD_MMC
    File file = SD_MMC.open(CONFIG_FILE_PATH_SD, FILE_WRITE);
    if (!file) {
        Serial.println("[CONFIG] Failed to open config file for writing on SD");
        free(json_string);
        return false;
    }
    
    file.print(json_string);
    file.close();
    free(json_string);
    
    Serial.println("[CONFIG] Config saved to SD");
    return true;
}

/**
 * @brief Exporte la config par defaut sur SD_MMC
 */
bool config_export_default_to_sd() {
    Serial.println("[CONFIG] Exporting default config to SD...");
    
    char json_buffer[2048];
    strcpy_P(json_buffer, DEFAULT_CONFIG_JSON);
    
    File file = SD_MMC.open("/config_default.json", FILE_WRITE);
    if (!file) {
        Serial.println("[CONFIG] Failed to create config_default.json");
        return false;
    }
    
    file.print(json_buffer);
    file.close();
    
    Serial.println("[CONFIG] Default config exported to /config_default.json");
    return true;
}

/**
 * @brief Obtient la configuration actuelle
 */
variometer_config_t* config_get() {
    return &g_config;
}

/**
 * @brief Definit une configuration par defaut hardcodee
 */
static void set_default_config() {
    // Logger
    strcpy(g_config.logger.output, "UART");
    strcpy(g_config.logger.kalman, "Info");
    strcpy(g_config.logger.i2c, "Warning");
    strcpy(g_config.logger.bmp390, "Info");
    strcpy(g_config.logger.imu, "Info");
    strcpy(g_config.logger.gps, "Info");
    strcpy(g_config.logger.theme, "Warning");
    strcpy(g_config.logger.display, "Info");
    strcpy(g_config.logger.map, "Info");
    strcpy(g_config.logger.wifi, "Info");
    strcpy(g_config.logger.storage, "Warning");
    strcpy(g_config.logger.flight, "Info");
    strcpy(g_config.logger.system, "Info");
    
    // Flight params
    g_config.flight_params.vario_damping = 0.5f;
    g_config.flight_params.vario_integration_time = 2.0f;
    g_config.flight_params.qnh = 1013.25f;
    g_config.flight_params.vario_threshold_strong = 3.0f;
    g_config.flight_params.vario_threshold_medium = 1.5f;
    g_config.flight_params.vario_threshold_weak = 0.5f;
    
    // Map
    g_config.map.zoom_default = 14;
    g_config.map.zoom_min = 10;
    g_config.map.zoom_max = 18;
    g_config.map.tile_max_age_days = 180;
    g_config.map.cache_size_mb = 100;
    
    // WiFi
    strcpy(g_config.wifi.ssid, "");
    strcpy(g_config.wifi.password, "");
    g_config.wifi.auto_connect = false;
    
    // Display
    g_config.display.brightness = 80;
    g_config.display.refresh_rate = 30;
    g_config.display.auto_brightness = false;
    
    g_config.config_source = CONFIG_SOURCE_HARDCODED;
}