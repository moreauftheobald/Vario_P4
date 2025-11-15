/**
 * @file config_loader.h
 * @brief Chargement de la configuration depuis JSON
 * 
 * Gere le chargement de la configuration depuis:
 * 1. Fichier config.json sur carte SD (prioritaire)
 * 2. Configuration par defaut en flash (fallback)
 * 
 * @author Theobald Moreau
 * @date 2025-11-14
 * @version 1.0
 * 
 * LOG DOCUMENTATION - MODULE: SYSTEM
 * [ERROR]
 *   - "Failed to parse config JSON" : Erreur parsing fichier config.json
 *   - "Config file corrupted" : Fichier config.json illisible
 *   - "Failed to mount LittleFS" : Impossible de monter LittleFS
 * [WARNING]
 *   - "Config file not found on SD, trying LittleFS" : Pas de config sur SD
 *   - "LittleFS config not found, using hardcoded" : Pas de config LittleFS
 *   - "Invalid value for %s, using default" : Valeur config invalide
 * [INFO]
 *   - "Config loaded from SD_MMC" : Config depuis carte SD
 *   - "Config loaded from LittleFS" : Config depuis flash (LittleFS)
 *   - "Config loaded from flash (hardcoded default)" : Config hardcodee
 *   - "Config saved to SD" : Configuration sauvegardee sur SD
 * [VERBOSE]
 *   - "Parsing config section: %s" : Detail parsing sections
 *   - "Config value: %s = %s" : Detail valeurs chargees
 */

#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>
#include <LittleFS.h>
#include "cJSON.h"
#include "../../data/config_data.h"
#include "../default_config.h"

// Chemins des fichiers de configuration
#define CONFIG_FILE_PATH_SD "/config.json"
#define CONFIG_FILE_PATH_LITTLEFS "/config.json"

/**
 * @brief Initialise le systeme de configuration
 * 
 * IMPORTANT: SD_MMC et LittleFS doivent etre initialises AVANT d'appeler
 * cette fonction. Exemple:
 * 
 *   SD_MMC.begin("/sdcard", true);  // Mode 1-bit
 *   LittleFS.begin(true);           // Auto-format si necessaire
 *   config_init();
 * 
 * Tente de charger la config dans cet ordre:
 * 1. Carte SD (SD_MMC)
 * 2. LittleFS (flash)
 * 3. Config hardcodee (default_config.h)
 * 
 * @return True si config chargee avec succes, false sinon
 */
bool config_init();

/**
 * @brief Charge la configuration depuis la carte SD (SD_MMC)
 * 
 * Lit le fichier config.json sur la carte SD et parse son contenu.
 * Met a jour la structure globale g_config.
 * 
 * @return True si chargement reussi, false sinon
 */
bool config_load_from_sd();

/**
 * @brief Charge la configuration depuis LittleFS
 * 
 * Lit le fichier config.json depuis LittleFS (flash) et parse son contenu.
 * Met a jour la structure globale g_config.
 * 
 * @return True si chargement reussi, false sinon
 */
bool config_load_from_littlefs();

/**
 * @brief Charge la configuration par defaut depuis le flash
 * 
 * Utilise la configuration JSON embarquee dans default_config.h.
 * Met a jour la structure globale g_config.
 * 
 * @return True si chargement reussi, false sinon
 */
bool config_load_from_flash();

/**
 * @brief Parse une configuration JSON et remplit la structure
 * 
 * Parse le contenu JSON et remplit la structure g_config.
 * Valide les valeurs et applique des limites si necessaire.
 * 
 * @param[in] json_string Chaine JSON a parser
 * 
 * @return True si parsing reussi, false sinon
 */
bool config_parse_json(const char* json_string);

/**
 * @brief Sauvegarde la configuration actuelle sur SD
 * 
 * Serialize la structure g_config en JSON et l'ecrit sur SD.
 * Utile pour sauvegarder les modifications faites via l'interface.
 * 
 * @return True si sauvegarde reussie, false sinon
 */
bool config_save_to_sd();

/**
 * @brief Exporte la configuration par defaut sur SD
 * 
 * Ecrit le fichier config_default.json sur SD avec la config
 * par defaut embarquee. Utile pour que l'utilisateur puisse
 * partir de cette base pour personnaliser.
 * 
 * @return True si export reussi, false sinon
 */
bool config_export_default_to_sd();

/**
 * @brief Obtient la configuration actuelle
 * 
 * @return Pointeur vers la structure de configuration globale
 */
variometer_config_t* config_get();

/**
 * @brief Valide une configuration
 * 
 * Verifie que toutes les valeurs sont dans des plages acceptables.
 * Corrige automatiquement les valeurs invalides.
 * 
 * @param[in,out] config Structure de configuration a valider
 * 
 * @return True si config valide (ou corrigee), false si erreur critique
 */
bool config_validate(variometer_config_t* config);

#endif // CONFIG_LOADER_H