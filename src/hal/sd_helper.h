/**
 * @file sd_helper.h
 * @brief Helper SD Card (SD_MMC 4-bit)
 * 
 * Fonctions pour gestion carte SD :
 * - Initialisation SD_MMC
 * - Lecture/Écriture fichiers
 * - Gestion répertoires
 * - Utilitaires (espace, listing, etc.)
 */

#ifndef SD_HELPER_H
#define SD_HELPER_H

#include <Arduino.h>
#include <SD_MMC.h>
#include "src/config/config.h"
#include "src/config/pins.h"
#include "src/system/logger.h"

// =============================================================================
// VARIABLES GLOBALES
// =============================================================================
static bool sd_initialized = false;
static SemaphoreHandle_t sd_mutex = NULL;

// =============================================================================
// CONSTANTES
// =============================================================================
#define SD_MOUNT_POINT "/sdcard"
#define SD_MAX_FILES 5  // Nombre max de fichiers ouverts simultanément

// =============================================================================
// INITIALISATION
// =============================================================================

/**
 * @brief Initialise la carte SD en mode SD_MMC 4-bit
 * @return true si succès
 */
bool sd_init() {
  LOG_I(LOG_SD, "Initializing SD Card (SD_MMC 4-bit)...");
  
  // Créer mutex
  if (sd_mutex == NULL) {
    sd_mutex = xSemaphoreCreateMutex();
    if (sd_mutex == NULL) {
      LOG_E(LOG_SD, "Failed to create SD mutex");
      return false;
    }
  }
  
  // Monter SD_MMC en mode 4-bit
  if (!SD_MMC.begin(SD_MOUNT_POINT, true)) {  // true = 4-bit mode
    LOG_E(LOG_SD, "SD Card mount failed");
    return false;
  }
  
  // Vérifier type de carte
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    LOG_E(LOG_SD, "No SD Card attached");
    return false;
  }
  
  // Log info carte
  const char* cardTypeStr = "UNKNOWN";
  switch (cardType) {
    case CARD_MMC:  cardTypeStr = "MMC"; break;
    case CARD_SD:   cardTypeStr = "SDSC"; break;
    case CARD_SDHC: cardTypeStr = "SDHC"; break;
    default: break;
  }
  
  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  uint64_t totalBytes = SD_MMC.totalBytes() / (1024 * 1024);
  uint64_t usedBytes = SD_MMC.usedBytes() / (1024 * 1024);
  
  LOG_I(LOG_SD, "SD Card Type: %s", cardTypeStr);
  LOG_I(LOG_SD, "SD Card Size: %llu MB", cardSize);
  LOG_I(LOG_SD, "Total space: %llu MB", totalBytes);
  LOG_I(LOG_SD, "Used space: %llu MB", usedBytes);
  
  sd_initialized = true;
  return true;
}

/**
 * @brief Démonte la carte SD proprement
 */
void sd_unmount() {
  if (sd_initialized) {
    SD_MMC.end();
    sd_initialized = false;
    LOG_I(LOG_SD, "SD Card unmounted");
  }
}

// =============================================================================
// UTILITAIRES
// =============================================================================

/**
 * @brief Obtient l'espace libre (en bytes)
 */
uint64_t sd_get_free_space() {
  if (!sd_initialized) return 0;
  return SD_MMC.totalBytes() - SD_MMC.usedBytes();
}

/**
 * @brief Obtient l'espace total (en bytes)
 */
uint64_t sd_get_total_space() {
  if (!sd_initialized) return 0;
  return SD_MMC.totalBytes();
}

/**
 * @brief Obtient l'espace utilisé (en bytes)
 */
uint64_t sd_get_used_space() {
  if (!sd_initialized) return 0;
  return SD_MMC.usedBytes();
}

// =============================================================================
// GESTION FICHIERS
// =============================================================================

/**
 * @brief Vérifie si un fichier existe
 */
bool sd_file_exists(const char* path) {
  if (!sd_initialized) return false;
  
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_SD, "Failed to take SD mutex");
    return false;
  }
  
  bool exists = SD_MMC.exists(path);
  
  xSemaphoreGive(sd_mutex);
  return exists;
}

/**
 * @brief Supprime un fichier
 */
bool sd_file_delete(const char* path) {
  if (!sd_initialized) return false;
  
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_SD, "Failed to take SD mutex");
    return false;
  }
  
  bool success = SD_MMC.remove(path);
  
  if (success) {
    LOG_I(LOG_SD, "Deleted: %s", path);
  } else {
    LOG_E(LOG_SD, "Failed to delete: %s", path);
  }
  
  xSemaphoreGive(sd_mutex);
  return success;
}

/**
 * @brief Renomme un fichier
 */
bool sd_file_rename(const char* old_path, const char* new_path) {
  if (!sd_initialized) return false;
  
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_SD, "Failed to take SD mutex");
    return false;
  }
  
  bool success = SD_MMC.rename(old_path, new_path);
  
  if (success) {
    LOG_I(LOG_SD, "Renamed: %s -> %s", old_path, new_path);
  } else {
    LOG_E(LOG_SD, "Failed to rename: %s", old_path);
  }
  
  xSemaphoreGive(sd_mutex);
  return success;
}

/**
 * @brief Obtient la taille d'un fichier (en bytes)
 */
size_t sd_file_size(const char* path) {
  if (!sd_initialized) return 0;
  
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_SD, "Failed to take SD mutex");
    return 0;
  }
  
  File file = SD_MMC.open(path, FILE_READ);
  if (!file) {
    LOG_E(LOG_SD, "Failed to open for size: %s", path);
    xSemaphoreGive(sd_mutex);
    return 0;
  }
  
  size_t size = file.size();
  file.close();
  
  xSemaphoreGive(sd_mutex);
  return size;
}

/**
 * @brief Lit un fichier entier en mémoire
 * @param path Chemin du fichier
 * @param buffer Buffer de sortie (alloué par l'appelant)
 * @param max_size Taille max à lire
 * @return Nombre de bytes lus, ou 0 si erreur
 */
size_t sd_file_read(const char* path, uint8_t* buffer, size_t max_size) {
  if (!sd_initialized || !buffer) return 0;
  
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_SD, "Failed to take SD mutex");
    return 0;
  }
  
  File file = SD_MMC.open(path, FILE_READ);
  if (!file) {
    LOG_E(LOG_SD, "Failed to open for reading: %s", path);
    xSemaphoreGive(sd_mutex);
    return 0;
  }
  
  size_t file_size = file.size();
  size_t to_read = (file_size < max_size) ? file_size : max_size;
  size_t bytes_read = file.read(buffer, to_read);
  
  file.close();
  xSemaphoreGive(sd_mutex);
  
  LOG_D(LOG_SD, "Read %zu bytes from %s", bytes_read, path);
  return bytes_read;
}

/**
 * @brief Écrit un fichier (remplace si existe)
 * @param path Chemin du fichier
 * @param data Données à écrire
 * @param size Taille des données
 * @return true si succès
 */
bool sd_file_write(const char* path, const uint8_t* data, size_t size) {
  if (!sd_initialized || !data) return false;
  
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_SD, "Failed to take SD mutex");
    return false;
  }
  
  File file = SD_MMC.open(path, FILE_WRITE);
  if (!file) {
    LOG_E(LOG_SD, "Failed to open for writing: %s", path);
    xSemaphoreGive(sd_mutex);
    return false;
  }
  
  size_t written = file.write(data, size);
  file.close();
  
  xSemaphoreGive(sd_mutex);
  
  if (written == size) {
    LOG_D(LOG_SD, "Wrote %zu bytes to %s", written, path);
    return true;
  } else {
    LOG_E(LOG_SD, "Write failed: %zu/%zu bytes to %s", written, size, path);
    return false;
  }
}

/**
 * @brief Ajoute des données à la fin d'un fichier
 * @param path Chemin du fichier
 * @param data Données à ajouter
 * @param size Taille des données
 * @return true si succès
 */
bool sd_file_append(const char* path, const uint8_t* data, size_t size) {
  if (!sd_initialized || !data) return false;
  
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_SD, "Failed to take SD mutex");
    return false;
  }
  
  File file = SD_MMC.open(path, FILE_APPEND);
  if (!file) {
    LOG_E(LOG_SD, "Failed to open for appending: %s", path);
    xSemaphoreGive(sd_mutex);
    return false;
  }
  
  size_t written = file.write(data, size);
  file.close();
  
  xSemaphoreGive(sd_mutex);
  
  if (written == size) {
    LOG_V(LOG_SD, "Appended %zu bytes to %s", written, path);
    return true;
  } else {
    LOG_E(LOG_SD, "Append failed: %zu/%zu bytes to %s", written, size, path);
    return false;
  }
}

/**
 * @brief Lit une ligne depuis un fichier texte
 * @param file Fichier ouvert
 * @param buffer Buffer de sortie
 * @param max_len Taille max du buffer
 * @return Nombre de caractères lus (sans '\n'), ou -1 si EOF
 */
int sd_file_read_line(File& file, char* buffer, size_t max_len) {
  if (!file || !buffer) return -1;
  
  size_t pos = 0;
  while (file.available() && pos < max_len - 1) {
    char c = file.read();
    if (c == '\n') break;
    if (c == '\r') continue;  // Ignorer CR
    buffer[pos++] = c;
  }
  
  buffer[pos] = '\0';
  
  if (pos == 0 && !file.available()) {
    return -1;  // EOF
  }
  
  return pos;
}

// =============================================================================
// GESTION RÉPERTOIRES
// =============================================================================

/**
 * @brief Vérifie si un répertoire existe
 */
bool sd_dir_exists(const char* path) {
  if (!sd_initialized) return false;
  
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_SD, "Failed to take SD mutex");
    return false;
  }
  
  File dir = SD_MMC.open(path);
  bool exists = false;
  
  if (dir) {
    exists = dir.isDirectory();
    dir.close();
  }
  
  xSemaphoreGive(sd_mutex);
  return exists;
}

/**
 * @brief Crée un répertoire (récursif)
 */
bool sd_mkdir(const char* path) {
  if (!sd_initialized) return false;
  
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_SD, "Failed to take SD mutex");
    return false;
  }
  
  bool success = SD_MMC.mkdir(path);
  
  if (success) {
    LOG_I(LOG_SD, "Created directory: %s", path);
  } else {
    LOG_E(LOG_SD, "Failed to create directory: %s", path);
  }
  
  xSemaphoreGive(sd_mutex);
  return success;
}

/**
 * @brief Supprime un répertoire (doit être vide)
 */
bool sd_rmdir(const char* path) {
  if (!sd_initialized) return false;
  
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_SD, "Failed to take SD mutex");
    return false;
  }
  
  bool success = SD_MMC.rmdir(path);
  
  if (success) {
    LOG_I(LOG_SD, "Deleted directory: %s", path);
  } else {
    LOG_E(LOG_SD, "Failed to delete directory: %s", path);
  }
  
  xSemaphoreGive(sd_mutex);
  return success;
}

/**
 * @brief Liste le contenu d'un répertoire
 * @param path Chemin du répertoire
 * @param callback Fonction appelée pour chaque entrée (name, is_dir, size)
 */
void sd_list_dir(const char* path, void (*callback)(const char* name, bool is_dir, size_t size)) {
  if (!sd_initialized || !callback) return;
  
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    LOG_E(LOG_SD, "Failed to take SD mutex");
    return;
  }
  
  File dir = SD_MMC.open(path);
  if (!dir || !dir.isDirectory()) {
    LOG_E(LOG_SD, "Failed to open directory: %s", path);
    xSemaphoreGive(sd_mutex);
    return;
  }
  
  File file = dir.openNextFile();
  while (file) {
    callback(file.name(), file.isDirectory(), file.size());
    file.close();
    file = dir.openNextFile();
  }
  
  dir.close();
  xSemaphoreGive(sd_mutex);
}

/**
 * @brief Callback exemple pour sd_list_dir (affiche dans Serial)
 */
void sd_list_dir_print_callback(const char* name, bool is_dir, size_t size) {
  if (is_dir) {
    LOG_I(LOG_SD, "  DIR : %s", name);
  } else {
    LOG_I(LOG_SD, "  FILE: %s (%zu bytes)", name, size);
  }
}

// =============================================================================
// UTILITAIRES AVANCÉS
// =============================================================================

/**
 * @brief Copie un fichier
 */
bool sd_file_copy(const char* src, const char* dst) {
  if (!sd_initialized) return false;
  
  LOG_I(LOG_SD, "Copying %s -> %s", src, dst);
  
  if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
    LOG_E(LOG_SD, "Failed to take SD mutex");
    return false;
  }
  
  File src_file = SD_MMC.open(src, FILE_READ);
  if (!src_file) {
    LOG_E(LOG_SD, "Failed to open source: %s", src);
    xSemaphoreGive(sd_mutex);
    return false;
  }
  
  File dst_file = SD_MMC.open(dst, FILE_WRITE);
  if (!dst_file) {
    LOG_E(LOG_SD, "Failed to open destination: %s", dst);
    src_file.close();
    xSemaphoreGive(sd_mutex);
    return false;
  }
  
  // Copie par blocs de 512 bytes
  uint8_t buffer[512];
  size_t total = 0;
  while (src_file.available()) {
    size_t read = src_file.read(buffer, sizeof(buffer));
    dst_file.write(buffer, read);
    total += read;
  }
  
  src_file.close();
  dst_file.close();
  xSemaphoreGive(sd_mutex);
  
  LOG_I(LOG_SD, "Copied %zu bytes", total);
  return true;
}

#endif  // SD_HELPER_H