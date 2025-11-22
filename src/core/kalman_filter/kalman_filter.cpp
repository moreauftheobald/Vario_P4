/**
 * @file kalman_filter.cpp
 * @brief Implémentation du filtre de Kalman
 */

#include "kalman_filter.h"
#include "src/system/logger/logger.h"
#include <math.h>

// Valeurs par défaut
#define DEFAULT_Q_ALTITUDE          0.01f   // Bruit processus altitude (m²)
#define DEFAULT_Q_VARIO             0.1f    // Bruit processus vario (m²/s²)
#define DEFAULT_R_BARO              0.5f    // Bruit mesure baro (m²)
#define DEFAULT_R_IMU               2.0f    // Bruit mesure IMU (m²/s⁴)
#define DEFAULT_MAX_INNOVATION_BARO 50.0f   // Rejeter outliers baro (m)
#define DEFAULT_MAX_INNOVATION_IMU  10.0f   // Rejeter outliers IMU (m/s²)
#define CONVERGENCE_THRESHOLD       1.0f    // P[0][0] < 1.0 = convergé
#define MIN_UPDATE_COUNT            20      // Min updates pour convergence

// ============================================================================
// INIT
// ============================================================================
void kalman_init(kalman_filter_t* filter, 
                 const kalman_config_t* config,
                 float initial_altitude, 
                 float initial_vario) {
    if (!filter) return;
    
    // État initial
    filter->altitude = initial_altitude;
    filter->vario = initial_vario;
    
    // Covariance initiale (grande incertitude)
    filter->P[0][0] = 100.0f;  // Altitude incertaine
    filter->P[0][1] = 0.0f;
    filter->P[1][0] = 0.0f;
    filter->P[1][1] = 10.0f;   // Vario incertain
    
    // Paramètres (utiliser config ou défauts)
    if (config) {
        filter->Q_altitude = config->Q_altitude;
        filter->Q_vario = config->Q_vario;
        filter->R_baro = config->R_baro;
        filter->R_imu = config->R_imu;
    } else {
        filter->Q_altitude = DEFAULT_Q_ALTITUDE;
        filter->Q_vario = DEFAULT_Q_VARIO;
        filter->R_baro = DEFAULT_R_BARO;
        filter->R_imu = DEFAULT_R_IMU;
    }
    
    // Init
    filter->last_update_ms = millis();
    filter->converged = false;
    filter->update_count = 0;
    filter->innovation_baro = 0.0f;
    filter->innovation_imu = 0.0f;
    
    LOG_I(LOG_MODULE_KALMAN, "Kalman initialized: alt=%.1fm vario=%.2fm/s", 
          initial_altitude, initial_vario);
}

// ============================================================================
// PREDICT (Prédiction)
// ============================================================================
void kalman_predict(kalman_filter_t* filter, 
                    float accel_vertical, 
                    float dt) {
    if (!filter) return;
    
    // Validation dt
    if (dt < 0.001f || dt > 1.0f) {
        LOG_E(LOG_MODULE_KALMAN, "Kalman dt invalid: %.3f", dt);
        return;
    }
    
    // ========== PRÉDICTION ÉTAT ==========
    // altitude(k+1) = altitude(k) + vario(k) * dt
    // vario(k+1) = vario(k) + accel_vertical * dt
    
    float altitude_pred = filter->altitude + filter->vario * dt;
    float vario_pred = filter->vario + accel_vertical * dt;
    
    // ========== PRÉDICTION COVARIANCE ==========
    // P(k+1) = F * P(k) * F' + Q
    //
    // Matrice de transition F :
    // F = | 1  dt |
    //     | 0   1 |
    //
    // Q = | Q_alt    0      |
    //     | 0        Q_vario |
    
    float P00 = filter->P[0][0];
    float P01 = filter->P[0][1];
    float P10 = filter->P[1][0];
    float P11 = filter->P[1][1];
    
    // P_pred = F * P * F'
    float P00_pred = P00 + 2.0f * dt * P01 + dt * dt * P11 + filter->Q_altitude;
    float P01_pred = P01 + dt * P11;
    float P10_pred = P10 + dt * P11;
    float P11_pred = P11 + filter->Q_vario;
    
    // Mettre à jour état
    filter->altitude = altitude_pred;
    filter->vario = vario_pred;
    
    // Mettre à jour covariance
    filter->P[0][0] = P00_pred;
    filter->P[0][1] = P01_pred;
    filter->P[1][0] = P10_pred;
    filter->P[1][1] = P11_pred;
    
    filter->last_update_ms = millis();
    
    LOG_V(LOG_MODULE_KALMAN, "Kalman predict: alt=%.1f vario=%.2f accel=%.2f dt=%.3f",
          filter->altitude, filter->vario, accel_vertical, dt);
}

// ============================================================================
// UPDATE BARO (Correction baromètre)
// ============================================================================
bool kalman_update_baro(kalman_filter_t* filter, float altitude_baro) {
    if (!filter) return false;
    
    // ========== INNOVATION ==========
    // y = z - H * x_pred
    // H = [1 0] (on mesure seulement l'altitude)
    
    float innovation = altitude_baro - filter->altitude;
    filter->innovation_baro = innovation;
    
    // Rejeter outliers
    if (fabsf(innovation) > DEFAULT_MAX_INNOVATION_BARO) {
        LOG_W(LOG_MODULE_KALMAN, "Baro measurement rejected: innovation=%.1fm", innovation);
        return false;
    }
    
    // ========== GAIN DE KALMAN ==========
    // S = H * P * H' + R
    // K = P * H' * S^-1
    
    float S = filter->P[0][0] + filter->R_baro;
    
    if (S < 0.0001f) {
        LOG_E(LOG_MODULE_KALMAN, "Kalman S too small: %.6f", S);
        return false;
    }
    
    float K0 = filter->P[0][0] / S;
    float K1 = filter->P[1][0] / S;
    
    // ========== CORRECTION ÉTAT ==========
    // x = x_pred + K * y
    
    filter->altitude += K0 * innovation;
    filter->vario += K1 * innovation;
    
    // ========== CORRECTION COVARIANCE ==========
    // P = (I - K * H) * P
    
    float P00 = filter->P[0][0];
    float P01 = filter->P[0][1];
    float P10 = filter->P[1][0];
    float P11 = filter->P[1][1];
    
    filter->P[0][0] = (1.0f - K0) * P00;
    filter->P[0][1] = (1.0f - K0) * P01;
    filter->P[1][0] = P10 - K1 * P00;
    filter->P[1][1] = P11 - K1 * P01;
    
    filter->update_count++;
    
    // Vérifier convergence
    if (!filter->converged && filter->update_count > MIN_UPDATE_COUNT) {
        if (filter->P[0][0] < CONVERGENCE_THRESHOLD) {
            filter->converged = true;
            LOG_I(LOG_MODULE_KALMAN, "Kalman converged after %d updates", 
                  filter->update_count);
        }
    }
    
    LOG_V(LOG_MODULE_KALMAN, "Kalman update baro: alt=%.1f vario=%.2f innovation=%.2f",
          filter->altitude, filter->vario, innovation);
    
    return true;
}

// ============================================================================
// UPDATE IMU (Correction accélération)
// ============================================================================
bool kalman_update_imu(kalman_filter_t* filter, float accel_vertical) {
    if (!filter) return false;
    
    // Cette fonction est optionnelle : l'accélération est déjà
    // intégrée dans la prédiction. On peut l'utiliser pour
    // corriger le vario directement si besoin.
    
    // ========== INNOVATION ==========
    // On mesure indirectement le vario via l'accélération
    // y = accel_measured - accel_predicted
    
    // Note : Dans un modèle complet, il faudrait estimer
    // l'accélération, mais ici on l'utilise comme entrée.
    
    // Pour l'instant, on skip cette correction car l'accel
    // est déjà dans predict(). À activer si besoin de correction
    // supplémentaire.
    
    return true;
}

// ============================================================================
// GETTERS
// ============================================================================
float kalman_get_altitude(const kalman_filter_t* filter) {
    return filter ? filter->altitude : 0.0f;
}

float kalman_get_vario(const kalman_filter_t* filter) {
    return filter ? filter->vario : 0.0f;
}

bool kalman_is_converged(const kalman_filter_t* filter) {
    return filter ? filter->converged : false;
}

float kalman_get_altitude_uncertainty(const kalman_filter_t* filter) {
    return filter ? sqrtf(filter->P[0][0]) : 0.0f;
}

float kalman_get_vario_uncertainty(const kalman_filter_t* filter) {
    return filter ? sqrtf(filter->P[1][1]) : 0.0f;
}

// ============================================================================
// RESET
// ============================================================================
void kalman_reset(kalman_filter_t* filter, float altitude, float vario) {
    if (!filter) return;
    
    filter->altitude = altitude;
    filter->vario = vario;
    
    // Reset covariance
    filter->P[0][0] = 100.0f;
    filter->P[0][1] = 0.0f;
    filter->P[1][0] = 0.0f;
    filter->P[1][1] = 10.0f;
    
    filter->converged = false;
    filter->update_count = 0;
    filter->last_update_ms = millis();
    
    LOG_I(LOG_MODULE_KALMAN, "Kalman reset: alt=%.1f vario=%.2f", altitude, vario);
}

// ============================================================================
// DEBUG
// ============================================================================
void kalman_print_state(const kalman_filter_t* filter) {
    if (!filter) return;
    
    LOG_I(LOG_MODULE_KALMAN, "=== Kalman State ===");
    LOG_I(LOG_MODULE_KALMAN, "Altitude: %.2f m (±%.2f m)", 
          filter->altitude, sqrtf(filter->P[0][0]));
    LOG_I(LOG_MODULE_KALMAN, "Vario: %.2f m/s (±%.2f m/s)", 
          filter->vario, sqrtf(filter->P[1][1]));
    LOG_I(LOG_MODULE_KALMAN, "Converged: %s (%d updates)", 
          filter->converged ? "YES" : "NO", filter->update_count);
    LOG_I(LOG_MODULE_KALMAN, "Last innovation: baro=%.2fm imu=%.2fm/s²",
          filter->innovation_baro, filter->innovation_imu);
}