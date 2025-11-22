/**
 * @file kalman_filter.h
 * @brief Filtre de Kalman pour fusion baromètre + IMU (variomètre)
 * 
 * Estime l'altitude et le vario en fusionnant :
 * - Baromètre BMP5 (altitude précise, lente, dérive thermique)
 * - IMU LSM6DSO32 (accélération verticale rapide, bruitée)
 * 
 * Modèle :
 *   État x = [altitude, vario]
 *   Prédiction : x(k+1) = F*x(k) + B*u(k)
 *   Correction : x(k) = x(k) + K*(z(k) - H*x(k))
 * 
 * @author Franck Moreau
 * @date 2025-11-23
 * @version 1.0
 * 
 * LOG DOCUMENTATION - MODULE: KALMAN
 * [ERROR]
 *   - "Kalman dt invalid: %.3f" : Delta temps aberrant
 *   - "Kalman innovation too large: %.2f" : Mesure incohérente
 * 
 * [WARNING]
 *   - "Kalman not converged yet" : Filtre en phase d'initialisation
 *   - "Baro measurement rejected" : Mesure baro rejetée (outlier)
 * 
 * [INFO]
 *   - "Kalman initialized" : Filtre initialisé
 *   - "Kalman reset" : Filtre réinitialisé
 * 
 * [VERBOSE]
 *   - "Kalman predict: alt=%.1f vario=%.2f" : Prédiction
 *   - "Kalman update baro: innovation=%.2f" : Correction baro
 *   - "Kalman update imu: innovation=%.3f" : Correction IMU
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>

/**
 * @brief Structure état du filtre de Kalman
 */
typedef struct {
    // État estimé
    float altitude;         // Altitude estimée (m)
    float vario;            // Vario estimé (m/s)
    
    // Matrice de covariance (2x2 symétrique)
    float P[2][2];          // Incertitude sur l'état
    
    // Bruit processus (tuning)
    float Q_altitude;       // Bruit sur altitude (m²)
    float Q_vario;          // Bruit sur vario (m²/s²)
    
    // Bruit mesures (tuning)
    float R_baro;           // Bruit baromètre (m²)
    float R_imu;            // Bruit IMU (m²/s⁴)
    
    // Timestamp
    uint32_t last_update_ms;
    
    // Convergence
    bool converged;         // Filtre a convergé
    uint32_t update_count;  // Nombre de mises à jour
    
    // Diagnostics
    float innovation_baro;  // Innovation dernière mesure baro
    float innovation_imu;   // Innovation dernière mesure IMU
    
} kalman_filter_t;

/**
 * @brief Configuration du filtre de Kalman
 */
typedef struct {
    // Bruit processus (à tuner selon comportement)
    float Q_altitude;       // Défaut: 0.01 (m²)
    float Q_vario;          // Défaut: 0.1 (m²/s²)
    
    // Bruit mesures (à tuner selon capteurs)
    float R_baro;           // Défaut: 0.5 (m²) - BMP5 précis
    float R_imu;            // Défaut: 2.0 (m²/s⁴) - IMU bruité
    
    // Seuils
    float max_innovation_baro;  // Défaut: 50.0m - Rejeter outliers
    float max_innovation_imu;   // Défaut: 10.0m/s² - Rejeter outliers
    
} kalman_config_t;

/**
 * @brief Initialise le filtre de Kalman
 * 
 * @param[out] filter Structure du filtre
 * @param[in] config Configuration (NULL = défauts)
 * @param[in] initial_altitude Altitude initiale (m)
 * @param[in] initial_vario Vario initial (m/s, typiquement 0)
 */
void kalman_init(kalman_filter_t* filter, 
                 const kalman_config_t* config,
                 float initial_altitude, 
                 float initial_vario);

/**
 * @brief Étape de prédiction (à appeler à chaque cycle)
 * 
 * Prédit l'état futur basé sur le modèle du système :
 *   altitude(k+1) = altitude(k) + vario(k) * dt
 *   vario(k+1) = vario(k) + accel_vertical(k) * dt
 * 
 * @param[in,out] filter Structure du filtre
 * @param[in] accel_vertical Accélération verticale IMU (m/s²)
 * @param[in] dt Delta temps depuis dernière prédiction (s)
 */
void kalman_predict(kalman_filter_t* filter, 
                    float accel_vertical, 
                    float dt);

/**
 * @brief Étape de correction avec mesure baromètre
 * 
 * Corrige l'altitude estimée avec la mesure du baromètre.
 * 
 * @param[in,out] filter Structure du filtre
 * @param[in] altitude_baro Altitude mesurée par baromètre (m)
 * @return true si correction appliquée, false si mesure rejetée
 */
bool kalman_update_baro(kalman_filter_t* filter, float altitude_baro);

/**
 * @brief Étape de correction avec mesure IMU (accélération)
 * 
 * Corrige le vario estimé avec l'accélération verticale.
 * Optionnel : peut être intégré directement dans predict.
 * 
 * @param[in,out] filter Structure du filtre
 * @param[in] accel_vertical Accélération verticale (m/s²)
 * @return true si correction appliquée, false si mesure rejetée
 */
bool kalman_update_imu(kalman_filter_t* filter, float accel_vertical);

/**
 * @brief Obtient l'altitude estimée
 * 
 * @param[in] filter Structure du filtre
 * @return Altitude en m
 */
float kalman_get_altitude(const kalman_filter_t* filter);

/**
 * @brief Obtient le vario estimé
 * 
 * @param[in] filter Structure du filtre
 * @return Vario en m/s
 */
float kalman_get_vario(const kalman_filter_t* filter);

/**
 * @brief Vérifie si le filtre a convergé
 * 
 * Convergé = covariance stabilisée (P faible)
 * 
 * @param[in] filter Structure du filtre
 * @return true si convergé
 */
bool kalman_is_converged(const kalman_filter_t* filter);

/**
 * @brief Obtient l'incertitude sur l'altitude
 * 
 * @param[in] filter Structure du filtre
 * @return Écart-type altitude (m)
 */
float kalman_get_altitude_uncertainty(const kalman_filter_t* filter);

/**
 * @brief Obtient l'incertitude sur le vario
 * 
 * @param[in] filter Structure du filtre
 * @return Écart-type vario (m/s)
 */
float kalman_get_vario_uncertainty(const kalman_filter_t* filter);

/**
 * @brief Reset le filtre (réinitialise état et covariance)
 * 
 * @param[in,out] filter Structure du filtre
 * @param[in] altitude Nouvelle altitude initiale (m)
 * @param[in] vario Nouveau vario initial (m/s)
 */
void kalman_reset(kalman_filter_t* filter, float altitude, float vario);

/**
 * @brief Affiche l'état du filtre (debug)
 * 
 * @param[in] filter Structure du filtre
 */
void kalman_print_state(const kalman_filter_t* filter);

#endif // KALMAN_FILTER_H