/**
 * @file config_loader.h
 * @brief Chargement de la configuration depuis JSON
 * 
 * Gère le chargement de la configuration selon priorité:
 * 1. Fichier config.json sur carte SD (prioritaire)
 * 2. Fichier config.json sur LittleFS (flash)
 * 3. Configuration hardcodée par défaut
 * 
 * LittleFS est initialisé automatiquement par ce module lors
 * du premier appel à config_load().
 * 
 * @author Theobald Moreau
 * @date 2025-11-15
 * @version 1.0
 * 
 * LOG DOCUMENTATION - MODULE: SYSTEM
 * [ERROR]
 *   - "Failed to parse config JSON" : Erreur parsing fichier config.json
 *   - "Config file corrupted" : Fichier config.json illisible
 *   - "Failed to mount LittleFS" : Impossible de monter LittleFS
 * 
 * [WARNING]
 *   - "Config file not found on SD" : Pas de config sur SD
 *   - "Config file not found in LittleFS" : Pas de config LittleFS
 *   - "Invalid value for %s, using default" : Valeur config invalide
 * 
 * [INFO]
 *   - "Configuration loaded from SD" : Config depuis carte SD
 *   - "Configuration loaded from LittleFS" : Config depuis flash
 *   - "Using hardcoded configuration" : Config hardcodée
 *   - "Configuration saved to SD" : Sauvegarde sur SD réussie
 *   - "Configuration saved to LittleFS" : Sauvegarde flash réussie
 *   - "LittleFS mounted" : LittleFS initialisé avec succès
 * 
 * [VERBOSE]
 *   - "Parsing config section: %s" : Détail parsing sections
 *   - "Config value: %s = %s" : Détail valeurs chargées
 */

#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <Arduino.h>
#include "../../data/config_data.h"

/**
 * @brief Charge la configuration selon priorité
 * 
 * Initialise automatiquement LittleFS si nécessaire.
 * Ordre de priorité:
 * 1. SD card (/config/config.json)
 * 2. LittleFS (/config.json)
 * 3. Configuration hardcodée
 * 
 * Cette fonction doit être appelée APRÈS sd_manager_init().
 * 
 * @return true si succès, false si erreur (utilise config hardcodée)
 */
bool config_load();

/**
 * @brief Valide une configuration
 * 
 * Vérifie que toutes les valeurs sont dans des plages acceptables.
 * Corrige automatiquement les valeurs invalides avec des défauts.
 * 
 * @param[in,out] config Structure de configuration à valider
 * @return true si config valide ou corrigée, false si erreur critique
 */
bool config_validate(variometer_config_t* config);

/**
 * @brief Sauvegarde la configuration sur SD
 * 
 * Sérialise g_config en JSON et l'écrit sur SD via sd_manager.
 * Utile pour sauvegarder les modifications faites via l'interface.
 * 
 * @return true si sauvegarde réussie, false si erreur
 */
bool config_save();

/**
 * @brief Sauvegarde la configuration sur LittleFS (flash)
 * 
 * Sérialise g_config en JSON et l'écrit sur LittleFS.
 * Utile pour avoir une config de secours en flash.
 * 
 * @return true si sauvegarde réussie, false si erreur
 */
bool config_save_to_littlefs();

#endif // CONFIG_LOADER_H