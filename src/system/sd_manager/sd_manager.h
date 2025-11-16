/**
 * @file sd_manager.h
 * @brief Gestionnaire centralisé de la carte SD
 * 
 * Gère l'initialisation, l'accès thread-safe et les opérations
 * sur la carte SD. Tous les modules utilisant la SD doivent passer
 * par ce gestionnaire.
 * 
 * @author Franck Moreau
 * @date 2025-11-15
 * @version 1.0
 * 
 * LOG DOCUMENTATION - MODULE: STORAGE
 * [ERROR]
 *   - "SD card initialization failed" : Échec init SD_MMC
 *   - "SD card not mounted" : Carte non montée lors d'une opération
 *   - "Failed to create directory: %s" : Impossible créer dossier
 *   - "Failed to open file: %s" : Impossible ouvrir fichier
 *   - "Failed to delete file: %s" : Impossible supprimer fichier
 * 
 * [WARNING]
 *   - "SD lock timeout" : Timeout acquisition mutex
 *   - "SD card removed during operation" : Carte retirée pendant opération
 * 
 * [INFO]
 *   - "SD card initialized: %lluMB" : Carte initialisée avec capacité
 *   - "SD card type: %s" : Type de carte détecté
 * 
 * [VERBOSE]
 *   - "SD lock acquired by %s" : Mutex acquis par module
 *   - "SD lock released by %s" : Mutex libéré par module
 *   - "Creating directory: %s" : Création dossier
 *   - "Opening file: %s mode: %s" : Ouverture fichier
 */

#ifndef SD_MANAGER_H
#define SD_MANAGER_H

#include <Arduino.h>
#include <SD_MMC.h>
#include <FS.h>

// Timeout par défaut pour acquisition du mutex (ms)
#define SD_LOCK_TIMEOUT_MS 1000

/**
 * @brief Initialise le gestionnaire SD et monte la carte
 * 
 * Configure SD_MMC en mode 4-bit, crée le mutex, monte la carte
 * et crée les dossiers systèmes si nécessaires.
 * 
 * Dossiers créés automatiquement:
 * - /logs : Fichiers de log
 * - /config : Fichiers de configuration
 * - /maps : Cache tiles OSM
 * - /flights : Traces IGC
 * 
 * @return true si succès, false si erreur
 */
bool sd_manager_init();

/**
 * @brief Vérifie si la carte SD est disponible
 * 
 * @return true si carte montée et accessible, false sinon
 */
bool sd_manager_is_available();

/**
 * @brief Acquiert le verrou d'accès à la SD (thread-safe)
 * 
 * Doit être appelé AVANT toute opération sur la SD.
 * Toujours appeler sd_manager_unlock() après usage.
 * 
 * @param[in] timeout_ms Timeout en millisecondes (0 = infini)
 * @return true si verrou acquis, false si timeout
 */
bool sd_manager_lock(uint32_t timeout_ms = SD_LOCK_TIMEOUT_MS);

/**
 * @brief Libère le verrou d'accès à la SD
 * 
 * Doit être appelé APRÈS chaque sd_manager_lock() réussi.
 */
void sd_manager_unlock();

/**
 * @brief Ouvre un fichier sur la SD
 * 
 * Wrapper thread-safe pour SD_MMC.open().
 * Le mutex DOIT être déjà acquis par l'appelant.
 * 
 * @param[in] path Chemin du fichier (ex: "/config/config.json")
 * @param[in] mode Mode d'ouverture (FILE_READ, FILE_WRITE, FILE_APPEND)
 * @return File Objet fichier (vérifier avec if(file) avant usage)
 */
File sd_manager_open(const char* path, const char* mode = FILE_READ);

/**
 * @brief Vérifie si un fichier ou dossier existe
 * 
 * Le mutex DOIT être déjà acquis par l'appelant.
 * 
 * @param[in] path Chemin à vérifier
 * @return true si existe, false sinon
 */
bool sd_manager_exists(const char* path);

/**
 * @brief Crée un dossier (et parents si nécessaire)
 * 
 * Le mutex DOIT être déjà acquis par l'appelant.
 * 
 * @param[in] path Chemin du dossier à créer
 * @return true si succès ou déjà existant, false si erreur
 */
bool sd_manager_mkdir(const char* path);

/**
 * @brief Supprime un fichier
 * 
 * Le mutex DOIT être déjà acquis par l'appelant.
 * 
 * @param[in] path Chemin du fichier à supprimer
 * @return true si succès, false si erreur
 */
bool sd_manager_remove(const char* path);

/**
 * @brief Renomme/déplace un fichier ou dossier
 * 
 * Le mutex DOIT être déjà acquis par l'appelant.
 * 
 * @param[in] path_from Chemin source
 * @param[in] path_to Chemin destination
 * @return true si succès, false si erreur
 */
bool sd_manager_rename(const char* path_from, const char* path_to);

/**
 * @brief Lit un fichier entier en mémoire
 * 
 * Fonction helper qui gère automatiquement le lock/unlock.
 * Attention à la taille mémoire disponible.
 * 
 * @param[in] path Chemin du fichier
 * @param[out] buffer Buffer alloué (à libérer avec free())
 * @param[out] size Taille du fichier lu
 * @return true si succès, false si erreur
 */
bool sd_manager_read_file(const char* path, char** buffer, size_t* size);

/**
 * @brief Écrit des données dans un fichier
 * 
 * Fonction helper qui gère automatiquement le lock/unlock.
 * Écrase le fichier existant.
 * 
 * @param[in] path Chemin du fichier
 * @param[in] data Données à écrire
 * @param[in] size Taille des données
 * @return true si succès, false si erreur
 */
bool sd_manager_write_file(const char* path, const char* data, size_t size);

/**
 * @brief Ajoute des données à la fin d'un fichier
 * 
 * Fonction helper qui gère automatiquement le lock/unlock.
 * 
 * @param[in] path Chemin du fichier
 * @param[in] data Données à ajouter
 * @param[in] size Taille des données
 * @return true si succès, false si erreur
 */
bool sd_manager_append_file(const char* path, const char* data, size_t size);

/**
 * @brief Obtient la taille d'un fichier
 * 
 * Le mutex DOIT être déjà acquis par l'appelant.
 * 
 * @param[in] path Chemin du fichier
 * @return Taille en octets, 0 si erreur ou fichier vide
 */
size_t sd_manager_file_size(const char* path);

/**
 * @brief Obtient l'espace libre sur la carte SD
 * 
 * @return Espace libre en octets
 */
uint64_t sd_manager_free_space();

/**
 * @brief Obtient la capacité totale de la carte SD
 * 
 * @return Capacité totale en octets
 */
uint64_t sd_manager_total_space();

/**
 * @brief Liste les fichiers d'un dossier
 * 
 * Fonction helper pour parcourir un dossier.
 * Le callback est appelé pour chaque fichier/dossier trouvé.
 * 
 * @param[in] path Chemin du dossier
 * @param[in] callback Fonction appelée pour chaque entrée (name, is_dir, size)
 * @return true si succès, false si erreur
 */
bool sd_manager_list_dir(const char* path, 
                         void (*callback)(const char* name, bool is_dir, size_t size));

/**
 * @brief Démonte proprement la carte SD
 * 
 * À appeler avant extinction ou retrait de la carte.
 */
void sd_manager_unmount();

#endif // SD_MANAGER_H