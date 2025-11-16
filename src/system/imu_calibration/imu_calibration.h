/**
 * @file imu_calibration.h
 * @brief Calibration de l'IMU LSM6DSO32
 * 
 * Gère la calibration des offsets gyroscope et accéléromètre.
 * Sauvegarde dans config.json (SD + LittleFS) avec validation qualité.
 * 
 * @author Theobald Moreau
 * @date 2025-11-15
 * @version 1.0
 * 
 * LOG DOCUMENTATION - MODULE: IMU
 * [ERROR]
 *   - "IMU not initialized for calibration" : Capteur non init
 *   - "Calibration failed - IMU moving" : Capteur bouge pendant calibration
 *   - "Calibration quality INVALID (score: %d)" : Calibration mauvaise
 *   - "Failed to save calibration to config" : Échec sauvegarde
 * 
 * [WARNING]
 *   - "Calibration quality ACCEPTABLE (score: %d)" : Calibration moyenne
 *   - "Temperature difference >10°C - accuracy reduced" : Écart température
 *   - "Calibration is old (>30 days)" : Calibration ancienne
 *   - "High GYRO drift detected: %.4f rad/s" : Dérive gyro élevée
 *   - "Gravity error: %.3f m/s²" : Erreur mesure gravité
 * 
 * [INFO]
 *   - "Calibration loaded from SD" : Calibration chargée SD
 *   - "Calibration loaded from LittleFS" : Calibration chargée flash
 *   - "Calibration quality: EXCELLENT (score: %d)" : Calibration parfaite
 *   - "Starting full calibration..." : Début calibration complète
 *   - "Calibration complete" : Calibration terminée
 *   - "Calibration saved to SD and LittleFS" : Sauvegarde OK
 * 
 * [VERBOSE]
 *   - "Gyro offsets: X=%.4f Y=%.4f Z=%.4f" : Offsets gyro calculés
 *   - "Accel offsets: X=%.3f Y=%.3f Z=%.3f" : Offsets accel calculés
 *   - "Accel scales: X=%.3f Y=%.3f Z=%.3f" : Gains accel calculés
 *   - "Gravity measured: %.3f m/s²" : Gravité mesurée
 *   - "Gyro drift: %.4f rad/s" : Dérive gyro mesurée
 */

#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <Arduino.h>

/**
 * @brief Structure de calibration IMU
 */
typedef struct {
    // Offsets gyroscope (rad/s)
    struct {
        float x, y, z;
    } gyro_offset;
    
    // Offsets accéléromètre (m/s²)
    struct {
        float x, y, z;
    } accel_offset;
    
    // Gains accéléromètre (facteur multiplicatif)
    struct {
        float x, y, z;
    } accel_scale;
    
    // Métadonnées
    struct {
        uint32_t timestamp;         // Timestamp calibration (millis)
        float temperature;          // Température lors calibration (°C)
        uint8_t quality_score;      // Score qualité 0-100
        char location[16];          // Source: "SD", "LittleFS", "None"
    } metadata;
} imu_calibration_t;

// Variable globale de calibration
extern imu_calibration_t g_imu_cal;

/**
 * @brief Charge la calibration depuis g_config
 * 
 * Charge les valeurs de calibration depuis la config globale
 * (qui a été chargée depuis SD ou LittleFS).
 * 
 * @return true si calibration chargée, false si pas de calibration
 */
bool imu_calibration_load_from_config();

/**
 * @brief Sauvegarde la calibration dans g_config
 * 
 * Copie les valeurs de g_imu_cal vers g_config pour
 * être sauvegardées via config_save().
 */
void imu_calibration_save_to_config();

/**
 * @brief Détecte si l'IMU est immobile
 * 
 * Analyse la variance du gyroscope et de l'accéléromètre
 * sur plusieurs échantillons pour détecter l'immobilité.
 * 
 * @return true si immobile, false si mouvement détecté
 */
bool imu_is_stationary();

/**
 * @brief Calibration rapide (gyro offset uniquement)
 * 
 * Mesure l'offset du gyroscope au repos (100 échantillons).
 * Rapide (~500ms) pour mise à jour au démarrage.
 * 
 * @return true si succès, false si erreur
 */
bool imu_calibration_quick_gyro();

/**
 * @brief Calibration complète (gyro + accel)
 * 
 * Mesure les offsets gyroscope et accéléromètre,
 * ainsi que les gains accéléromètre (200 échantillons).
 * 
 * L'IMU doit être posé à plat et immobile.
 * 
 * @return true si succès, false si erreur
 */
bool imu_calibration_full();

/**
 * @brief Valide la qualité de la calibration
 * 
 * Effectue plusieurs tests :
 * - Test gravité (norme accéléromètre)
 * - Test dérive gyroscope
 * - Test température
 * - Test âge calibration
 * 
 * @return Score de qualité 0-100 (≥60 = acceptable)
 */
int imu_calibration_validate();

/**
 * @brief Test de la mesure de gravité
 * 
 * Vérifie que la norme de l'accéléromètre calibré
 * est proche de 9.81 m/s².
 * 
 * @return Score 0-100
 */
int imu_calibration_check_gravity();

/**
 * @brief Test de la dérive gyroscope
 * 
 * Vérifie que le gyroscope calibré est proche de zéro
 * lorsque l'IMU est immobile.
 * 
 * @return Score 0-100
 */
int imu_calibration_check_gyro_drift();

/**
 * @brief Test de la température
 * 
 * Compare la température actuelle avec celle de calibration.
 * Un écart >10°C peut réduire la précision.
 * 
 * @return true si température similaire, false si écart important
 */
bool imu_calibration_check_temperature();

/**
 * @brief Test de l'âge de la calibration
 * 
 * Vérifie que la calibration n'est pas trop ancienne.
 * >30 jours = warning, >90 jours = error
 * 
 * @return true si calibration récente, false si trop vieille
 */
bool imu_calibration_check_age();

/**
 * @brief Applique la calibration aux données brutes
 * 
 * Soustrait les offsets et applique les gains aux données
 * brutes de l'IMU.
 * 
 * @param[in,out] gx Gyro X (rad/s)
 * @param[in,out] gy Gyro Y (rad/s)
 * @param[in,out] gz Gyro Z (rad/s)
 * @param[in,out] ax Accel X (m/s²)
 * @param[in,out] ay Accel Y (m/s²)
 * @param[in,out] az Accel Z (m/s²)
 */
void imu_calibration_apply(float* gx, float* gy, float* gz,
                           float* ax, float* ay, float* az);

/**
 * @brief Réinitialise la calibration aux valeurs par défaut
 * 
 * Offsets à zéro, gains à 1.0.
 * Utilisé si aucune calibration n'est disponible.
 */
void imu_calibration_reset();

/**
 * @brief Affiche la calibration actuelle
 * 
 * Log les valeurs de calibration pour debug.
 */
void imu_calibration_print();

#endif // IMU_CALIBRATION_H