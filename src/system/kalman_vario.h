/**
 * @file kalman_vario.h
 * @brief Filtre de Kalman pour fusion IMU + Baro + GPS
 * 
 * État: [altitude, vitesse_verticale]
 * Entrée: accélération verticale (IMU via quaternions)
 * Mesures: altitude_baro, altitude_gps
 */

#ifndef KALMAN_VARIO_H
#define KALMAN_VARIO_H

#include <Arduino.h>
#include "src/system/logger.h"

// =============================================================================
// STRUCTURE DU FILTRE
// =============================================================================
struct KalmanVario {
  // État [altitude, vitesse_verticale]
  float altitude;      // Altitude estimée (m)
  float vario;         // Vitesse verticale estimée (m/s)
  
  // Matrice de covariance P (2x2)
  float P[2][2];
  
  // Bruit de processus Q
  float Q_acc;         // Bruit accélération
  float Q_vario;       // Bruit vario
  
  // Bruit de mesure R
  float R_baro;        // Bruit altitude baro
  float R_gps;         // Bruit altitude GPS
  
  // Timestamp
  unsigned long last_update_us;
};

// =============================================================================
// FONCTIONS
// =============================================================================

/**
 * @brief Initialise le filtre de Kalman
 */
void kalman_init(KalmanVario* k, float alt_init) {
  k->altitude = alt_init;
  k->vario = 0.0f;
  
  // Covariance initiale
  k->P[0][0] = 10.0f;   // Variance altitude
  k->P[0][1] = 0.0f;
  k->P[1][0] = 0.0f;
  k->P[1][1] = 1.0f;    // Variance vario
  
  // Bruit de processus
  k->Q_acc = 0.1f;      // Bruit accélération IMU
  k->Q_vario = 0.01f;   // Bruit dérive vario
  
  // Bruit de mesure
  k->R_baro = 0.5f;     // Baro très précis
  k->R_gps = 10.0f;     // GPS moins précis
  
  k->last_update_us = micros();
  
  LOG_I(LOG_KALMAN, "Kalman initialized at alt=%.1fm", alt_init);
}

/**
 * @brief Extrait l'accélération verticale depuis quaternion et accélération
 * @param qw, qx, qy, qz Quaternion (orientation)
 * @param ax, ay, az Accélération brute (m/s²)
 * @return Accélération verticale dans le repère monde (m/s²)
 */
float kalman_get_vertical_accel(float qw, float qx, float qy, float qz,
                                 float ax, float ay, float az) {
  // Rotation de l'accélération du repère capteur vers repère monde
  // en utilisant le quaternion conjugué
  float ax_world = 2.0f * (qw*ax*qw + qy*ax*qz - qz*ax*qy + qx*ax*qx);
  float ay_world = 2.0f * (qw*ay*qw + qz*ay*qx - qx*ay*qz + qy*ay*qy);
  float az_world = 2.0f * (qw*az*qw + qx*az*qy - qy*az*qx + qz*az*qz);
  
  // Composante verticale (axe Z dans repère monde)
  // Soustraire gravité (9.81 m/s²)
  return az_world - 9.81f;
}

/**
 * @brief Prédiction du filtre (étape temporelle)
 * @param k Filtre
 * @param acc_vertical Accélération verticale (m/s²)
 * @param dt Delta temps (s)
 */
void kalman_predict(KalmanVario* k, float acc_vertical, float dt) {
  // Prédiction de l'état
  // altitude(k+1) = altitude(k) + vario(k) * dt + 0.5 * acc * dt²
  // vario(k+1) = vario(k) + acc * dt
  
  k->altitude += k->vario * dt + 0.5f * acc_vertical * dt * dt;
  k->vario += acc_vertical * dt;
  
  // Matrice de transition F
  float F[2][2] = {
    {1.0f, dt},
    {0.0f, 1.0f}
  };
  
  // Prédiction de la covariance: P = F*P*F' + Q
  float P_new[2][2];
  
  // P_new = F * P
  P_new[0][0] = F[0][0]*k->P[0][0] + F[0][1]*k->P[1][0];
  P_new[0][1] = F[0][0]*k->P[0][1] + F[0][1]*k->P[1][1];
  P_new[1][0] = F[1][0]*k->P[0][0] + F[1][1]*k->P[1][0];
  P_new[1][1] = F[1][0]*k->P[0][1] + F[1][1]*k->P[1][1];
  
  // P_new = P_new * F'
  k->P[0][0] = P_new[0][0]*F[0][0] + P_new[0][1]*F[0][1];
  k->P[0][1] = P_new[0][0]*F[1][0] + P_new[0][1]*F[1][1];
  k->P[1][0] = P_new[1][0]*F[0][0] + P_new[1][1]*F[0][1];
  k->P[1][1] = P_new[1][0]*F[1][0] + P_new[1][1]*F[1][1];
  
  // Ajouter bruit de processus Q
  k->P[0][0] += k->Q_acc * dt * dt * dt * dt / 4.0f;
  k->P[0][1] += k->Q_acc * dt * dt * dt / 2.0f;
  k->P[1][0] += k->Q_acc * dt * dt * dt / 2.0f;
  k->P[1][1] += k->Q_acc * dt * dt + k->Q_vario * dt;
}

/**
 * @brief Correction avec mesure d'altitude
 * @param k Filtre
 * @param altitude_measured Altitude mesurée (m)
 * @param R Variance de la mesure
 */
void kalman_update(KalmanVario* k, float altitude_measured, float R) {
  // Innovation
  float y = altitude_measured - k->altitude;
  
  // Innovation covariance: S = H*P*H' + R
  // H = [1, 0] (on mesure l'altitude)
  float S = k->P[0][0] + R;
  
  // Gain de Kalman: K = P*H' / S
  float K[2];
  K[0] = k->P[0][0] / S;
  K[1] = k->P[1][0] / S;
  
  // Correction de l'état
  k->altitude += K[0] * y;
  k->vario += K[1] * y;
  
  // Correction de la covariance: P = (I - K*H) * P
  float P_new[2][2];
  P_new[0][0] = (1.0f - K[0]) * k->P[0][0];
  P_new[0][1] = (1.0f - K[0]) * k->P[0][1];
  P_new[1][0] = k->P[1][0] - K[1] * k->P[0][0];
  P_new[1][1] = k->P[1][1] - K[1] * k->P[0][1];
  
  k->P[0][0] = P_new[0][0];
  k->P[0][1] = P_new[0][1];
  k->P[1][0] = P_new[1][0];
  k->P[1][1] = P_new[1][1];
}

/**
 * @brief Mise à jour complète du filtre
 * @param k Filtre
 * @param acc_vertical Accélération verticale IMU (m/s²)
 * @param alt_baro Altitude baromètre (m)
 * @param alt_gps Altitude GPS (m), mettre NAN si pas de fix
 */
void kalman_process(KalmanVario* k, float acc_vertical, float alt_baro, float alt_gps) {
  // Calcul dt
  unsigned long now_us = micros();
  float dt = (now_us - k->last_update_us) / 1000000.0f;
  k->last_update_us = now_us;
  
  // Prédiction
  kalman_predict(k, acc_vertical, dt);
  
  // Correction avec baro (toujours disponible)
  kalman_update(k, alt_baro, k->R_baro);
  
  // Correction avec GPS (si disponible)
  if (!isnan(alt_gps)) {
    kalman_update(k, alt_gps, k->R_gps);
  }
  
  LOG_V(LOG_KALMAN, "Alt=%.1fm Vario=%.2fm/s AccZ=%.2f dt=%.3fs", 
        k->altitude, k->vario, acc_vertical, dt);
}

#endif  // KALMAN_VARIO_H