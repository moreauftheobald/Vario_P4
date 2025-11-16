/**
 * @file task_flight.h
 * @brief Tâche unifiée capteurs + fusion + Kalman + calculs vol
 * 
 * Pipeline complet temps réel à 200 Hz :
 * 1. Lecture capteurs (LSM6DSO32 200Hz, BMP390 50Hz, GPS 2Hz)
 * 2. Calibration IMU appliquée
 * 3. Fusion Madgwick (200 Hz) → quaternions
 * 4. Extraction accélération verticale
 * 5. Filtre Kalman (100 Hz) → altitude + vario
 * 6. Calculs paramètres vol
 * 7. Stockage dans flight_data (protégé mutex)
 * 
 * @author Theobald Moreau
 * @date 2025-11-15
 * @version 1.0
 * 
 * LOG DOCUMENTATION - MODULE: FLIGHT
 * [ERROR]
 *   - "Task flight creation failed" : Échec création tâche FreeRTOS
 *   - "Flight data mutex creation failed" : Échec création mutex
 *   - "LSM6DSO32 read error" : Erreur lecture IMU
 *   - "BMP390 read error" : Erreur lecture baromètre
 * 
 * [WARNING]
 *   - "Flight task overrun: %d ms" : Tâche trop lente (>5ms)
 *   - "GPS no fix after %d seconds" : Pas de fix GPS
 * 
 * [INFO]
 *   - "Flight task started @ %d Hz" : Tâche démarrée
 *   - "GPS fix acquired: %d satellites" : Fix GPS obtenu
 *   - "Flight parameters initialized" : Paramètres vol initialisés
 * 
 * [VERBOSE]
 *   - "Vario: %.2f m/s, Alt: %.1f m" : Données vario
 *   - "Quaternion: [%.3f %.3f %.3f %.3f]" : Quaternions fusion
 *   - "GPS: %.6f %.6f alt %.1f" : Position GPS
 */

#ifndef TASK_FLIGHT_H
#define TASK_FLIGHT_H

#include <Arduino.h>
#include <Adafruit_AHRS.h>
#include "src/data/flight_data.h"

// Instance globale du filtre Madgwick
extern Adafruit_Madgwick filter_madgwick;

// Mutex protection flight_data
extern SemaphoreHandle_t flight_data_mutex;

/**
 * @brief Crée et démarre la tâche flight
 * 
 * Initialise :
 * - Mutex flight_data
 * - Filtre Madgwick
 * - Filtre Kalman
 * - Structure flight_data
 * - Tâche FreeRTOS
 * 
 * @return true si succès, false si erreur
 */
bool task_flight_init();

/**
 * @brief Point d'entrée de la tâche FreeRTOS
 * 
 * Boucle principale à 200 Hz.
 * 
 * @param[in] params Paramètres tâche (non utilisé)
 */
void task_flight_run(void* params);

/**
 * @brief Lit les capteurs et applique calibration
 * 
 * Lecture à 200 Hz :
 * - LSM6DSO32 : accéléromètre + gyroscope
 * - BMP390 : pression + température (50 Hz, diviseur /4)
 * - GPS : position (2 Hz, diviseur /100)
 * 
 * Applique calibration IMU aux données brutes.
 */
void task_flight_read_sensors();

/**
 * @brief Fusion Madgwick des données IMU
 * 
 * Fusionne accéléromètre + gyroscope pour obtenir
 * l'orientation sous forme de quaternions.
 * 
 * Fréquence : 200 Hz
 */
void task_flight_fusion_madgwick();

/**
 * @brief Extrait l'accélération verticale calibrée
 * 
 * Utilise les quaternions pour transformer l'accélération
 * du référentiel capteur vers le référentiel terrestre,
 * puis soustrait la gravité pour obtenir l'accélération
 * verticale pure.
 * 
 * @return Accélération verticale (m/s²), positive = montée
 */
float task_flight_get_vertical_acceleration();

/**
 * @brief Filtre de Kalman altitude/vario
 * 
 * Fusionne :
 * - Altitude baromètre (mesure)
 * - Accélération verticale IMU (prédiction)
 * 
 * Produit :
 * - Altitude filtrée (m)
 * - Vitesse verticale (vario) filtrée (m/s)
 * 
 * Fréquence : 100 Hz (diviseur /2)
 */
void task_flight_kalman_filter();

/**
 * @brief Calcule les paramètres de vol
 * 
 * À partir des données fusionnées, calcule :
 * - Altitudes (GPS, QNE, QNH, QFE)
 * - Vario instantané, intégré, moyenné
 * - Vitesses (GPS, air estimée)
 * - Distances (parcourue, depuis décollage)
 * - Finesse
 * - Temps de vol
 * 
 * Fréquence : 100 Hz (ou 10 Hz pour certains calculs)
 */
void task_flight_compute_parameters();

/**
 * @brief Stocke les résultats dans flight_data (mutex)
 * 
 * Copie toutes les données calculées dans la structure
 * globale flight_data protégée par mutex.
 * 
 * Les autres tâches peuvent lire flight_data de manière
 * thread-safe avec task_flight_get_data().
 */
void task_flight_store_data();

/**
 * @brief Obtient une copie thread-safe de flight_data
 * 
 * Utilisée par les autres tâches (display, storage, etc.)
 * pour lire les données de vol sans risque de corruption.
 * 
 * @param[out] data Pointeur vers structure à remplir
 * @return true si succès, false si mutex timeout
 */
bool task_flight_get_data(flight_data_t* data);

/**
 * @brief Initialise le filtre de Kalman
 * 
 * Configure les matrices de covariance et les bruits
 * selon les constantes de config.h.
 */
void task_flight_kalman_init();

/**
 * @brief Réinitialise les calculs de vol
 * 
 * Appelé au décollage ou à la demande utilisateur.
 * Remet à zéro : distances, temps de vol, gains, etc.
 */
void task_flight_reset();

#endif // TASK_FLIGHT_H