/**
 * @file usb_msc_manager.h
 * @brief Gestionnaire USB Mass Storage pour carte SD
 * 
 * Expose la carte SD via le port USB-A en mode MSC (Mass Storage Class).
 * Permet la modification facile des fichiers config.json, theme.json, etc.
 * 
 * ATTENTION: Lorsque la SD est exposée en USB, elle n'est plus accessible
 * par l'ESP32. Il faut démonter l'USB avant de pouvoir la réutiliser.
 * 
 * @author Franck Moreau
 * @date 2025-11-19
 * @version 1.0
 * 
 * LOG DOCUMENTATION - MODULE: SYSTEM
 * [ERROR]
 *   - "USB MSC init failed" : Échec init USB MSC
 *   - "SD not available for USB" : SD non disponible
 *   - "USB MSC already active" : Tentative double init
 * 
 * [WARNING]
 *   - "USB MSC stopped while active" : Arrêt pendant utilisation
 *   - "SD access blocked by USB" : Tentative accès SD pendant USB actif
 * 
 * [INFO]
 *   - "USB MSC started" : USB MSC activé
 *   - "USB MSC stopped" : USB MSC désactivé
 *   - "SD remounted after USB" : SD remontée après USB
 * 
 * [VERBOSE]
 *   - "USB MSC state change: %d" : Changement état USB
 */

#ifndef USB_MSC_MANAGER_H
#define USB_MSC_MANAGER_H

#include <Arduino.h>
#include <USB.h>
#include <USBMSC.h>
#include <SD_MMC.h>

// Instance USB MSC
extern USBMSC MSC;

/**
 * @brief État du gestionnaire USB MSC
 */
typedef enum {
    USB_MSC_IDLE = 0,      // Inactif
    USB_MSC_ACTIVE,        // Actif (SD exposée en USB)
    USB_MSC_ERROR          // Erreur
} usb_msc_state_t;

/**
 * @brief Initialise le gestionnaire USB MSC
 * 
 * Configure l'USB MSC mais ne l'active pas encore.
 * Permet d'activer/désactiver à la demande.
 * 
 * @return true si succès, false si erreur
 */
bool usb_msc_init();

/**
 * @brief Active l'exposition USB de la carte SD
 * 
 * ATTENTION: 
 * - Démonte la SD du côté ESP32
 * - La SD devient accessible UNIQUEMENT via USB
 * - Tous les accès ESP32 à la SD échoueront
 * - Appeler usb_msc_stop() pour reprendre l'accès ESP32
 * 
 * @return true si succès, false si erreur
 */
bool usb_msc_start();

/**
 * @brief Désactive l'exposition USB et remonte la SD
 * 
 * - Arrête l'USB MSC
 * - Remonte la SD pour accès ESP32
 * - Recrée les dossiers système si nécessaire
 * 
 * @return true si succès, false si erreur
 */
bool usb_msc_stop();

/**
 * @brief Vérifie si l'USB MSC est actif
 * 
 * @return true si USB actif, false sinon
 */
bool usb_msc_is_active();

/**
 * @brief Obtient l'état actuel du gestionnaire
 * 
 * @return État USB MSC
 */
usb_msc_state_t usb_msc_get_state();

/**
 * @brief Toggle USB MSC (activer/désactiver)
 * 
 * Fonction helper pour bouton on/off.
 * Si inactif → active
 * Si actif → désactive
 * 
 * @return true si changement réussi, false si erreur
 */
bool usb_msc_toggle();

#endif // USB_MSC_MANAGER_H