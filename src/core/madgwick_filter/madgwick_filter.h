/**
 * @file madgwick_filter.h
 * @brief Filtre de Madgwick pour fusion IMU 6 axes
 * 
 * Fusionne accéléromètre + gyroscope pour estimer l'orientation
 * sous forme de quaternion. Permet de calculer l'accélération
 * verticale quelle que soit l'orientation du variomètre.
 * 
 * Basé sur l'algorithme de Sebastian Madgwick (2010)
 * 
 * @author Franck Moreau
 * @date 2025-11-23
 * @version 1.0
 */

#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include <Arduino.h>
#include <math.h>

/**
 * @brief Structure quaternion (orientation 3D)
 */
typedef struct {
  float w, x, y, z;
} quaternion_t;

/**
 * @brief Structure angles d'Euler (orientation 3D)
 */
typedef struct {
  float roll;   // Roulis (rotation axe X) en degrés
  float pitch;  // Tangage (rotation axe Y) en degrés
  float yaw;    // Lacet (rotation axe Z) en degrés
} euler_t;

/**
 * @brief Structure vecteur 3D
 */
typedef struct {
  float x, y, z;
} vector3_t;

/**
 * @brief Structure filtre de Madgwick
 */
typedef struct {
  quaternion_t q;        // Quaternion d'orientation
  float beta;            // Gain du filtre (typique: 0.1)
  float sample_freq;     // Fréquence d'échantillonnage (Hz)
  uint32_t last_update;  // Timestamp dernière mise à jour (ms)
} madgwick_filter_t;

/**
 * @brief Initialise le quaternion depuis l'accéléromètre
 */
void madgwick_init_from_accel(madgwick_filter_t* filter, float ax, float ay, float az);

/**
 * @brief Initialise le filtre de Madgwick
 * 
 * @param[out] filter Structure du filtre
 * @param[in] sample_freq Fréquence échantillonnage (Hz, ex: 100)
 * @param[in] beta Gain du filtre (0.033-0.1, défaut: 0.1)
 */
void madgwick_init(madgwick_filter_t* filter, float sample_freq, float beta);

/**
 * @brief Met à jour le filtre avec nouvelles données IMU
 * 
 * @param[in,out] filter Structure du filtre
 * @param[in] gx Gyroscope X (rad/s)
 * @param[in] gy Gyroscope Y (rad/s)
 * @param[in] gz Gyroscope Z (rad/s)
 * @param[in] ax Accéléromètre X (m/s²)
 * @param[in] ay Accéléromètre Y (m/s²)
 * @param[in] az Accéléromètre Z (m/s²)
 */
void madgwick_update(madgwick_filter_t* filter,
                     float gx, float gy, float gz,
                     float ax, float ay, float az);

/**
 * @brief Convertit le quaternion en angles d'Euler
 * 
 * @param[in] q Quaternion
 * @param[out] euler Angles d'Euler (degrés)
 */
void madgwick_quaternion_to_euler(const quaternion_t* q, euler_t* euler);

/**
 * @brief Calcule l'accélération dans le référentiel terrestre
 * 
 * Transforme l'accélération mesurée (référentiel capteur)
 * vers le référentiel terrestre (orientation-indépendante).
 * 
 * @param[in] filter Filtre (contient quaternion)
 * @param[in] ax Accel X capteur (m/s²)
 * @param[in] ay Accel Y capteur (m/s²)
 * @param[in] az Accel Z capteur (m/s²)
 * @param[out] earth_accel Accel dans référentiel terrestre
 */
void madgwick_get_earth_accel(const madgwick_filter_t* filter,
                              float ax, float ay, float az,
                              vector3_t* earth_accel);

/**
 * @brief Obtient l'accélération verticale (orientation-indépendante)
 * 
 * Calcule la composante verticale de l'accélération après
 * transformation dans le référentiel terrestre et soustraction
 * de la gravité.
 * 
 * @param[in] filter Filtre (contient quaternion)
 * @param[in] ax Accel X capteur (m/s²)
 * @param[in] ay Accel Y capteur (m/s²)
 * @param[in] az Accel Z capteur (m/s²)
 * @return Accélération verticale en m/s² (+ = haut, - = bas)
 */
float madgwick_get_vertical_accel(const madgwick_filter_t* filter,
                                  float ax, float ay, float az);

/**
 * @brief Reset le filtre (quaternion = identité)
 */
void madgwick_reset(madgwick_filter_t* filter);

/**
 * @brief TEST : Rotation d'un vecteur par quaternion (pour debug)
 * 
 * @param[in] q Quaternion
 * @param[in] vx, vy, vz Composantes vecteur
 * @param[out] rx, ry, rz Composantes résultat
 */
void madgwick_rotate_vector_test(const quaternion_t* q, 
                                  float vx, float vy, float vz,
                                  float* rx, float* ry, float* rz);

#endif  // MADGWICK_FILTER_H