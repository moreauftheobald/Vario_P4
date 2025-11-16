/**
 * @file imu_calibration.cpp
 * @brief Implémentation calibration IMU
 * 
 * @author Theobald Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#include "imu_calibration.h"
#include "src/system/sensor_init/sensor_init.h"
#include "src/system/logger/logger.h"
#include "src/data/config_data.h"
#include "config/config.h"
#include <math.h>

// Variable globale de calibration
imu_calibration_t g_imu_cal = {0};

/**
 * @brief Charge calibration depuis g_config
 */
bool imu_calibration_load_from_config() {
    // Vérifier si une calibration existe dans config
    if (g_config.imu_calibration.metadata.quality_score == 0) {
        LOG_W(LOG_MODULE_IMU, "No calibration found in config");
        imu_calibration_reset();
        return false;
    }
    
    // Copier depuis g_config vers g_imu_cal
    g_imu_cal.gyro_offset.x = g_config.imu_calibration.gyro_offset.x;
    g_imu_cal.gyro_offset.y = g_config.imu_calibration.gyro_offset.y;
    g_imu_cal.gyro_offset.z = g_config.imu_calibration.gyro_offset.z;
    
    g_imu_cal.accel_offset.x = g_config.imu_calibration.accel_offset.x;
    g_imu_cal.accel_offset.y = g_config.imu_calibration.accel_offset.y;
    g_imu_cal.accel_offset.z = g_config.imu_calibration.accel_offset.z;
    
    g_imu_cal.accel_scale.x = g_config.imu_calibration.accel_scale.x;
    g_imu_cal.accel_scale.y = g_config.imu_calibration.accel_scale.y;
    g_imu_cal.accel_scale.z = g_config.imu_calibration.accel_scale.z;
    
    g_imu_cal.metadata.timestamp = g_config.imu_calibration.metadata.timestamp;
    g_imu_cal.metadata.temperature = g_config.imu_calibration.metadata.temperature;
    g_imu_cal.metadata.quality_score = g_config.imu_calibration.metadata.quality_score;
    strncpy(g_imu_cal.metadata.location, 
            g_config.imu_calibration.metadata.location, 16);
    
    LOG_I(LOG_MODULE_IMU, "Calibration loaded from %s", g_imu_cal.metadata.location);
    
    return true;
}

/**
 * @brief Sauvegarde calibration dans g_config
 */
void imu_calibration_save_to_config() {
    // Copier depuis g_imu_cal vers g_config
    g_config.imu_calibration.gyro_offset.x = g_imu_cal.gyro_offset.x;
    g_config.imu_calibration.gyro_offset.y = g_imu_cal.gyro_offset.y;
    g_config.imu_calibration.gyro_offset.z = g_imu_cal.gyro_offset.z;
    
    g_config.imu_calibration.accel_offset.x = g_imu_cal.accel_offset.x;
    g_config.imu_calibration.accel_offset.y = g_imu_cal.accel_offset.y;
    g_config.imu_calibration.accel_offset.z = g_imu_cal.accel_offset.z;
    
    g_config.imu_calibration.accel_scale.x = g_imu_cal.accel_scale.x;
    g_config.imu_calibration.accel_scale.y = g_imu_cal.accel_scale.y;
    g_config.imu_calibration.accel_scale.z = g_imu_cal.accel_scale.z;
    
    g_config.imu_calibration.metadata.timestamp = g_imu_cal.metadata.timestamp;
    g_config.imu_calibration.metadata.temperature = g_imu_cal.metadata.temperature;
    g_config.imu_calibration.metadata.quality_score = g_imu_cal.metadata.quality_score;
    strncpy(g_config.imu_calibration.metadata.location,
            g_imu_cal.metadata.location, 16);
}

/**
 * @brief Détecte si IMU immobile
 */
bool imu_is_stationary() {
    if (!sensor_lsm6dso32_ready) {
        return false;
    }
    
    float gyro_variance = 0;
    float accel_variance = 0;
    
    for (int i = 0; i < IMU_STATIONARY_SAMPLES; i++) {
        sensors_event_t accel, gyro, temp;
        lsm6dso32.getEvent(&accel, &gyro, &temp);
        
        // Variance gyroscope (valeur absolue totale)
        gyro_variance += fabs(gyro.gyro.x) + fabs(gyro.gyro.y) + fabs(gyro.gyro.z);
        
        // Variance accéléromètre (écart par rapport à gravité attendue)
        float accel_magnitude = sqrt(accel.acceleration.x * accel.acceleration.x +
                                     accel.acceleration.y * accel.acceleration.y +
                                     accel.acceleration.z * accel.acceleration.z);
        accel_variance += fabs(accel_magnitude - GRAVITY_STANDARD);
        
        delay(5);
    }
    
    gyro_variance /= IMU_STATIONARY_SAMPLES;
    accel_variance /= IMU_STATIONARY_SAMPLES;
    
    bool is_still = (gyro_variance < IMU_STATIONARY_GYRO_THRESHOLD) && 
                    (accel_variance < IMU_STATIONARY_ACCEL_THRESHOLD);
    
    // Debug : toujours afficher pour voir les valeurs
    LOG_I(LOG_MODULE_IMU, "Stationary check: gyro=%.4f (<%0.2f?) accel=%.4f (<%0.2f?) -> %s",
          gyro_variance, IMU_STATIONARY_GYRO_THRESHOLD,
          accel_variance, IMU_STATIONARY_ACCEL_THRESHOLD,
          is_still ? "STILL" : "MOVING");
    
    return is_still;
}

/**
 * @brief Calibration rapide gyro
 */
bool imu_calibration_quick_gyro() {
    if (!sensor_lsm6dso32_ready) {
        LOG_E(LOG_MODULE_IMU, "IMU not initialized for calibration");
        return false;
    }
    
    LOG_I(LOG_MODULE_IMU, "Quick gyro calibration...");
    
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    
    for (int i = 0; i < IMU_CAL_SAMPLES_QUICK; i++) {
        sensors_event_t accel, gyro, temp;
        lsm6dso32.getEvent(&accel, &gyro, &temp);
        
        gx_sum += gyro.gyro.x;
        gy_sum += gyro.gyro.y;
        gz_sum += gyro.gyro.z;
        
        delay(5);
    }
    
    // Calculer moyennes
    g_imu_cal.gyro_offset.x = gx_sum / IMU_CAL_SAMPLES_QUICK;
    g_imu_cal.gyro_offset.y = gy_sum / IMU_CAL_SAMPLES_QUICK;
    g_imu_cal.gyro_offset.z = gz_sum / IMU_CAL_SAMPLES_QUICK;
    
    LOG_V(LOG_MODULE_IMU, "Gyro offsets: X=%.4f Y=%.4f Z=%.4f",
          g_imu_cal.gyro_offset.x, g_imu_cal.gyro_offset.y, g_imu_cal.gyro_offset.z);
    
    return true;
}

/**
 * @brief Calibration complète
 */
bool imu_calibration_full() {
    if (!sensor_lsm6dso32_ready) {
        LOG_E(LOG_MODULE_IMU, "IMU not initialized for calibration");
        return false;
    }
    
    LOG_I(LOG_MODULE_IMU, "Starting full calibration...");
    
    // Vérifier immobilité
    if (!imu_is_stationary()) {
        LOG_E(LOG_MODULE_IMU, "Calibration failed - IMU moving");
        return false;
    }
    
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float temp_sum = 0;
    
    // Collecter échantillons
    for (int i = 0; i < IMU_CAL_SAMPLES_FULL; i++) {
        sensors_event_t accel, gyro, temp;
        lsm6dso32.getEvent(&accel, &gyro, &temp);
        
        gx_sum += gyro.gyro.x;
        gy_sum += gyro.gyro.y;
        gz_sum += gyro.gyro.z;
        
        ax_sum += accel.acceleration.x;
        ay_sum += accel.acceleration.y;
        az_sum += accel.acceleration.z;
        
        temp_sum += temp.temperature;
        
        delay(5);
    }
    
    // Calculer offsets gyroscope
    g_imu_cal.gyro_offset.x = gx_sum / IMU_CAL_SAMPLES_FULL;
    g_imu_cal.gyro_offset.y = gy_sum / IMU_CAL_SAMPLES_FULL;
    g_imu_cal.gyro_offset.z = gz_sum / IMU_CAL_SAMPLES_FULL;
    
    // Calculer offsets accéléromètre
    g_imu_cal.accel_offset.x = ax_sum / IMU_CAL_SAMPLES_FULL;
    g_imu_cal.accel_offset.y = ay_sum / IMU_CAL_SAMPLES_FULL;
    g_imu_cal.accel_offset.z = (az_sum / IMU_CAL_SAMPLES_FULL) - GRAVITY_STANDARD;
    
    // Gains par défaut (calibration 6-faces nécessaire pour gains précis)
    g_imu_cal.accel_scale.x = 1.0f;
    g_imu_cal.accel_scale.y = 1.0f;
    g_imu_cal.accel_scale.z = 1.0f;
    
    // Métadonnées
    g_imu_cal.metadata.timestamp = millis();
    g_imu_cal.metadata.temperature = temp_sum / IMU_CAL_SAMPLES_FULL;
    g_imu_cal.metadata.quality_score = 0; // Sera calculé par validate()
    strncpy(g_imu_cal.metadata.location, "New", 16);
    
    LOG_V(LOG_MODULE_IMU, "Gyro offsets: X=%.4f Y=%.4f Z=%.4f",
          g_imu_cal.gyro_offset.x, g_imu_cal.gyro_offset.y, g_imu_cal.gyro_offset.z);
    LOG_V(LOG_MODULE_IMU, "Accel offsets: X=%.3f Y=%.3f Z=%.3f",
          g_imu_cal.accel_offset.x, g_imu_cal.accel_offset.y, g_imu_cal.accel_offset.z);
    
    LOG_I(LOG_MODULE_IMU, "Calibration complete");
    
    return true;
}

/**
 * @brief Valide qualité calibration
 */
int imu_calibration_validate() {
    LOG_I(LOG_MODULE_IMU, "Validating calibration quality...");
    
    int gravity_score = imu_calibration_check_gravity();
    int gyro_score = imu_calibration_check_gyro_drift();
    bool temp_ok = imu_calibration_check_temperature();
    bool age_ok = imu_calibration_check_age();
    
    // Score global (moyenne gravity + gyro)
    int overall_score = (gravity_score + gyro_score) / 2;
    
    // Pénalité si température ou âge problématique
    if (!temp_ok) overall_score -= 10;
    if (!age_ok) overall_score -= 10;
    
    // Limiter à 0-100
    if (overall_score < 0) overall_score = 0;
    if (overall_score > 100) overall_score = 100;
    
    // Sauvegarder score
    g_imu_cal.metadata.quality_score = overall_score;
    
    // Log résultat
    if (overall_score >= IMU_CAL_SCORE_EXCELLENT) {
        LOG_I(LOG_MODULE_IMU, "Calibration quality: EXCELLENT (score: %d/100)", overall_score);
    } else if (overall_score >= IMU_CAL_SCORE_GOOD) {
        LOG_I(LOG_MODULE_IMU, "Calibration quality: GOOD (score: %d/100)", overall_score);
    } else if (overall_score >= IMU_CAL_SCORE_ACCEPTABLE) {
        LOG_W(LOG_MODULE_IMU, "Calibration quality: ACCEPTABLE (score: %d/100)", overall_score);
    } else {
        LOG_E(LOG_MODULE_IMU, "Calibration quality INVALID (score: %d/100)", overall_score);
    }
    
    return overall_score;
}

/**
 * @brief Test gravité
 */
int imu_calibration_check_gravity() {
    if (!sensor_lsm6dso32_ready) return 0;
    
    float gravity_sum = 0;
    
    for (int i = 0; i < 100; i++) {
        sensors_event_t accel, gyro, temp;
        lsm6dso32.getEvent(&accel, &gyro, &temp);
        
        // Appliquer calibration
        float ax = (accel.acceleration.x - g_imu_cal.accel_offset.x) * g_imu_cal.accel_scale.x;
        float ay = (accel.acceleration.y - g_imu_cal.accel_offset.y) * g_imu_cal.accel_scale.y;
        float az = (accel.acceleration.z - g_imu_cal.accel_offset.z) * g_imu_cal.accel_scale.z;
        
        float norm = sqrt(ax*ax + ay*ay + az*az);
        gravity_sum += norm;
        
        delay(2);
    }
    
    float measured_gravity = gravity_sum / 100.0f;
    float error = fabs(measured_gravity - GRAVITY_STANDARD);
    
    LOG_V(LOG_MODULE_IMU, "Gravity measured: %.3f m/s² (error: %.3f m/s²)",
          measured_gravity, error);
    
    if (error > IMU_CAL_GRAVITY_TOLERANCE_GOOD) {
        LOG_W(LOG_MODULE_IMU, "Gravity error: %.3f m/s²", error);
    }
    
    // Score
    if (error < IMU_CAL_GRAVITY_TOLERANCE_EXCELLENT) {
        return 100;
    } else if (error < IMU_CAL_GRAVITY_TOLERANCE_GOOD) {
        return 70;
    } else {
        return 30;
    }
}

/**
 * @brief Test dérive gyro
 */
int imu_calibration_check_gyro_drift() {
    if (!sensor_lsm6dso32_ready) return 0;
    
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    
    for (int i = 0; i < 100; i++) {
        sensors_event_t accel, gyro, temp;
        lsm6dso32.getEvent(&accel, &gyro, &temp);
        
        // Appliquer calibration
        float gx = gyro.gyro.x - g_imu_cal.gyro_offset.x;
        float gy = gyro.gyro.y - g_imu_cal.gyro_offset.y;
        float gz = gyro.gyro.z - g_imu_cal.gyro_offset.z;
        
        gx_sum += fabs(gx);
        gy_sum += fabs(gy);
        gz_sum += fabs(gz);
        
        delay(2);
    }
    
    float avg_drift = (gx_sum + gy_sum + gz_sum) / 300.0f;
    
    LOG_V(LOG_MODULE_IMU, "Gyro drift: %.4f rad/s", avg_drift);
    
    if (avg_drift > IMU_CAL_GYRO_DRIFT_GOOD) {
        LOG_W(LOG_MODULE_IMU, "High GYRO drift detected: %.4f rad/s", avg_drift);
    }
    
    // Score
    if (avg_drift < IMU_CAL_GYRO_DRIFT_EXCELLENT) {
        return 100;
    } else if (avg_drift < IMU_CAL_GYRO_DRIFT_GOOD) {
        return 70;
    } else {
        return 30;
    }
}

/**
 * @brief Test température
 */
bool imu_calibration_check_temperature() {
    if (!sensor_lsm6dso32_ready) return false;
    
    sensors_event_t accel, gyro, temp;
    lsm6dso32.getEvent(&accel, &gyro, &temp);
    
    float current_temp = temp.temperature;
    float cal_temp = g_imu_cal.metadata.temperature;
    float temp_diff = fabs(current_temp - cal_temp);
    
    LOG_V(LOG_MODULE_IMU, "Temperature: %.1f°C (cal: %.1f°C, diff: %.1f°C)",
          current_temp, cal_temp, temp_diff);
    
    if (temp_diff > IMU_CAL_TEMP_DIFF_WARNING) {
        LOG_W(LOG_MODULE_IMU, "Temperature difference >10°C - accuracy reduced");
        return false;
    }
    
    return true;
}

/**
 * @brief Test âge calibration
 */
bool imu_calibration_check_age() {
    uint32_t age_ms = millis() - g_imu_cal.metadata.timestamp;
    uint32_t age_days = age_ms / 86400000;
    
    LOG_V(LOG_MODULE_IMU, "Calibration age: %d days", age_days);
    
    if (age_days > IMU_CAL_AGE_ERROR_DAYS) {
        LOG_W(LOG_MODULE_IMU, "Calibration is old (>90 days)");
        return false;
    } else if (age_days > IMU_CAL_AGE_WARNING_DAYS) {
        LOG_W(LOG_MODULE_IMU, "Calibration is old (>30 days)");
        return true;
    }
    
    return true;
}

/**
 * @brief Applique calibration
 */
void imu_calibration_apply(float* gx, float* gy, float* gz,
                           float* ax, float* ay, float* az) {
    // Gyroscope (soustraire offsets)
    *gx -= g_imu_cal.gyro_offset.x;
    *gy -= g_imu_cal.gyro_offset.y;
    *gz -= g_imu_cal.gyro_offset.z;
    
    // Accéléromètre (soustraire offsets puis appliquer gains)
    *ax = (*ax - g_imu_cal.accel_offset.x) * g_imu_cal.accel_scale.x;
    *ay = (*ay - g_imu_cal.accel_offset.y) * g_imu_cal.accel_scale.y;
    *az = (*az - g_imu_cal.accel_offset.z) * g_imu_cal.accel_scale.z;
}

/**
 * @brief Reset calibration
 */
void imu_calibration_reset() {
    memset(&g_imu_cal, 0, sizeof(imu_calibration_t));
    
    g_imu_cal.accel_scale.x = 1.0f;
    g_imu_cal.accel_scale.y = 1.0f;
    g_imu_cal.accel_scale.z = 1.0f;
    
    strncpy(g_imu_cal.metadata.location, "None", 16);
    
    LOG_W(LOG_MODULE_IMU, "Calibration reset to defaults");
}

/**
 * @brief Affiche calibration
 */
void imu_calibration_print() {
    LOG_I(LOG_MODULE_IMU, "=== IMU Calibration ===");
    LOG_I(LOG_MODULE_IMU, "Gyro offsets: X=%.4f Y=%.4f Z=%.4f rad/s",
          g_imu_cal.gyro_offset.x, g_imu_cal.gyro_offset.y, g_imu_cal.gyro_offset.z);
    LOG_I(LOG_MODULE_IMU, "Accel offsets: X=%.3f Y=%.3f Z=%.3f m/s²",
          g_imu_cal.accel_offset.x, g_imu_cal.accel_offset.y, g_imu_cal.accel_offset.z);
    LOG_I(LOG_MODULE_IMU, "Accel scales: X=%.3f Y=%.3f Z=%.3f",
          g_imu_cal.accel_scale.x, g_imu_cal.accel_scale.y, g_imu_cal.accel_scale.z);
    LOG_I(LOG_MODULE_IMU, "Quality: %d/100, Temp: %.1f°C, Source: %s",
          g_imu_cal.metadata.quality_score,
          g_imu_cal.metadata.temperature,
          g_imu_cal.metadata.location);
    LOG_I(LOG_MODULE_IMU, "======================");
}