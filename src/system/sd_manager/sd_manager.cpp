/**
 * @file sd_manager.cpp
 * @brief Implémentation du gestionnaire SD
 * 
 * @author Theobald Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#include "sd_manager.h"
#include "../logger/logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Variables globales privées
static bool sd_initialized = false;
static SemaphoreHandle_t sd_mutex = NULL;

// Dossiers système à créer automatiquement
static const char* SYSTEM_DIRS[] = {
    "/logs",
    "/config",
    "/maps",
    "/flights"
};
static const int NUM_SYSTEM_DIRS = 4;

/**
 * @brief Crée les dossiers système nécessaires
 */
static bool create_system_directories() {
    for (int i = 0; i < NUM_SYSTEM_DIRS; i++) {
        if (!SD_MMC.exists(SYSTEM_DIRS[i])) {
            LOG_V(LOG_MODULE_STORAGE, "Creating directory: %s", SYSTEM_DIRS[i]);
            if (!SD_MMC.mkdir(SYSTEM_DIRS[i])) {
                LOG_E(LOG_MODULE_STORAGE, "Failed to create directory: %s", SYSTEM_DIRS[i]);
                return false;
            }
        }
    }
    return true;
}

/**
 * @brief Obtient le nom du type de carte
 */
static const char* get_card_type_name(uint8_t card_type) {
    switch (card_type) {
        case CARD_NONE: return "None";
        case CARD_MMC:  return "MMC";
        case CARD_SD:   return "SDSC";
        case CARD_SDHC: return "SDHC";
        default:        return "Unknown";
    }
}

/**
 * @brief Initialise le gestionnaire SD
 */
bool sd_manager_init() {
    // Créer le mutex si pas déjà fait
    if (sd_mutex == NULL) {
        sd_mutex = xSemaphoreCreateMutex();
        if (sd_mutex == NULL) {
            Serial.println("[SD_MANAGER] Failed to create mutex");
            return false;
        }
    }
    
    // Initialiser SD_MMC en mode 4-bit
    if (!SD_MMC.begin("/sdcard", false, true)) {
        LOG_E(LOG_MODULE_STORAGE, "SD card initialization failed");
        sd_initialized = false;
        return false;
    }
    
    // Vérifier type de carte
    uint8_t card_type = SD_MMC.cardType();
    if (card_type == CARD_NONE) {
        LOG_E(LOG_MODULE_STORAGE, "No SD card detected");
        sd_initialized = false;
        return false;
    }
    
    // Obtenir infos carte
    uint64_t card_size = SD_MMC.cardSize() / (1024 * 1024); // En MB
    
    LOG_I(LOG_MODULE_STORAGE, "SD card initialized: %lluMB", card_size);
    LOG_I(LOG_MODULE_STORAGE, "SD card type: %s", get_card_type_name(card_type));
    
    // Créer les dossiers système
    if (!create_system_directories()) {
        LOG_W(LOG_MODULE_STORAGE, "Failed to create some system directories");
        // Continue quand même, pas bloquant
    }
    
    sd_initialized = true;
    return true;
}

/**
 * @brief Vérifie disponibilité SD
 */
bool sd_manager_is_available() {
    return sd_initialized;
}

/**
 * @brief Acquiert le verrou SD
 */
bool sd_manager_lock(uint32_t timeout_ms) {
    if (sd_mutex == NULL) {
        return false;
    }
    
    TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    
    if (xSemaphoreTake(sd_mutex, timeout_ticks) == pdTRUE) {
        LOG_V(LOG_MODULE_STORAGE, "SD lock acquired");
        return true;
    }
    
    LOG_W(LOG_MODULE_STORAGE, "SD lock timeout");
    return false;
}

/**
 * @brief Libère le verrou SD
 */
void sd_manager_unlock() {
    if (sd_mutex != NULL) {
        xSemaphoreGive(sd_mutex);
        LOG_V(LOG_MODULE_STORAGE, "SD lock released");
    }
}

/**
 * @brief Ouvre un fichier
 */
File sd_manager_open(const char* path, const char* mode) {
    if (!sd_initialized) {
        LOG_E(LOG_MODULE_STORAGE, "SD card not mounted");
        return File();
    }
    
    LOG_V(LOG_MODULE_STORAGE, "Opening file: %s mode: %s", path, mode);
    
    File file = SD_MMC.open(path, mode);
    if (!file) {
        LOG_E(LOG_MODULE_STORAGE, "Failed to open file: %s", path);
    }
    
    return file;
}

/**
 * @brief Vérifie si un fichier existe
 */
bool sd_manager_exists(const char* path) {
    if (!sd_initialized) {
        return false;
    }
    
    return SD_MMC.exists(path);
}

/**
 * @brief Crée un dossier
 */
bool sd_manager_mkdir(const char* path) {
    if (!sd_initialized) {
        LOG_E(LOG_MODULE_STORAGE, "SD card not mounted");
        return false;
    }
    
    // Vérifier si existe déjà
    if (SD_MMC.exists(path)) {
        return true;
    }
    
    LOG_V(LOG_MODULE_STORAGE, "Creating directory: %s", path);
    
    if (!SD_MMC.mkdir(path)) {
        LOG_E(LOG_MODULE_STORAGE, "Failed to create directory: %s", path);
        return false;
    }
    
    return true;
}

/**
 * @brief Supprime un fichier
 */
bool sd_manager_remove(const char* path) {
    if (!sd_initialized) {
        LOG_E(LOG_MODULE_STORAGE, "SD card not mounted");
        return false;
    }
    
    if (!SD_MMC.remove(path)) {
        LOG_E(LOG_MODULE_STORAGE, "Failed to delete file: %s", path);
        return false;
    }
    
    return true;
}

/**
 * @brief Renomme un fichier
 */
bool sd_manager_rename(const char* path_from, const char* path_to) {
    if (!sd_initialized) {
        LOG_E(LOG_MODULE_STORAGE, "SD card not mounted");
        return false;
    }
    
    if (!SD_MMC.rename(path_from, path_to)) {
        LOG_E(LOG_MODULE_STORAGE, "Failed to rename %s to %s", path_from, path_to);
        return false;
    }
    
    return true;
}

/**
 * @brief Lit un fichier complet
 */
bool sd_manager_read_file(const char* path, char** buffer, size_t* size) {
    if (!sd_manager_lock()) {
        return false;
    }
    
    File file = sd_manager_open(path, FILE_READ);
    if (!file) {
        sd_manager_unlock();
        return false;
    }
    
    *size = file.size();
    *buffer = (char*)malloc(*size + 1); // +1 pour null terminator
    
    if (*buffer == NULL) {
        LOG_E(LOG_MODULE_STORAGE, "Failed to allocate memory for file: %s", path);
        file.close();
        sd_manager_unlock();
        return false;
    }
    
    size_t bytes_read = file.read((uint8_t*)*buffer, *size);
    (*buffer)[*size] = '\0'; // Null terminator
    
    file.close();
    sd_manager_unlock();
    
    if (bytes_read != *size) {
        LOG_E(LOG_MODULE_STORAGE, "Read size mismatch for file: %s", path);
        free(*buffer);
        *buffer = NULL;
        return false;
    }
    
    return true;
}

/**
 * @brief Écrit un fichier
 */
bool sd_manager_write_file(const char* path, const char* data, size_t size) {
    if (!sd_manager_lock()) {
        return false;
    }
    
    File file = sd_manager_open(path, FILE_WRITE);
    if (!file) {
        sd_manager_unlock();
        return false;
    }
    
    size_t bytes_written = file.write((const uint8_t*)data, size);
    file.close();
    sd_manager_unlock();
    
    if (bytes_written != size) {
        LOG_E(LOG_MODULE_STORAGE, "Write size mismatch for file: %s", path);
        return false;
    }
    
    return true;
}

/**
 * @brief Ajoute à un fichier
 */
bool sd_manager_append_file(const char* path, const char* data, size_t size) {
    if (!sd_manager_lock()) {
        return false;
    }
    
    File file = sd_manager_open(path, FILE_APPEND);
    if (!file) {
        sd_manager_unlock();
        return false;
    }
    
    size_t bytes_written = file.write((const uint8_t*)data, size);
    file.close();
    sd_manager_unlock();
    
    if (bytes_written != size) {
        LOG_E(LOG_MODULE_STORAGE, "Append size mismatch for file: %s", path);
        return false;
    }
    
    return true;
}

/**
 * @brief Obtient la taille d'un fichier
 */
size_t sd_manager_file_size(const char* path) {
    if (!sd_initialized) {
        return 0;
    }
    
    File file = SD_MMC.open(path, FILE_READ);
    if (!file) {
        return 0;
    }
    
    size_t size = file.size();
    file.close();
    
    return size;
}

/**
 * @brief Obtient l'espace libre
 */
uint64_t sd_manager_free_space() {
    if (!sd_initialized) {
        return 0;
    }
    
    return SD_MMC.totalBytes() - SD_MMC.usedBytes();
}

/**
 * @brief Obtient l'espace total
 */
uint64_t sd_manager_total_space() {
    if (!sd_initialized) {
        return 0;
    }
    
    return SD_MMC.totalBytes();
}

/**
 * @brief Liste les fichiers d'un dossier
 */
bool sd_manager_list_dir(const char* path, 
                         void (*callback)(const char* name, bool is_dir, size_t size)) {
    if (!sd_manager_lock()) {
        return false;
    }
    
    File dir = SD_MMC.open(path);
    if (!dir || !dir.isDirectory()) {
        LOG_E(LOG_MODULE_STORAGE, "Failed to open directory: %s", path);
        sd_manager_unlock();
        return false;
    }
    
    File file = dir.openNextFile();
    while (file) {
        callback(file.name(), file.isDirectory(), file.size());
        file = dir.openNextFile();
    }
    
    dir.close();
    sd_manager_unlock();
    
    return true;
}

/**
 * @brief Démonte la carte SD
 */
void sd_manager_unmount() {
    if (sd_initialized) {
        SD_MMC.end();
        sd_initialized = false;
        LOG_I(LOG_MODULE_STORAGE, "SD card unmounted");
    }
    
    if (sd_mutex != NULL) {
        vSemaphoreDelete(sd_mutex);
        sd_mutex = NULL;
    }
}