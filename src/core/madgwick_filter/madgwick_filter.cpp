/**
 * @file madgwick_filter.cpp
 * @brief Implémentation du filtre de Madgwick
 */

#include "madgwick_filter.h"

#define GRAVITY_EARTH 9.80665f  // m/s²

// ============================================================================
// FONCTIONS HELPER (privées)
// ============================================================================

/**
 * @brief Normalise un quaternion
 */
static void quaternion_normalize(quaternion_t* q) {
    float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    
    if (norm > 0.0001f) {
        float inv_norm = 1.0f / norm;
        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    }
}

/**
 * @brief Rotation d'un vecteur par un quaternion
 * 
 * v' = q * v * q^-1
 */
static void quaternion_rotate_vector(const quaternion_t* q, const vector3_t* v, vector3_t* result) {
    // Formule optimisée : v' = v + 2*cross(q.xyz, cross(q.xyz, v) + q.w*v)
    
    float qx2 = q->x * 2.0f;
    float qy2 = q->y * 2.0f;
    float qz2 = q->z * 2.0f;
    
    float qw2 = q->w * 2.0f;
    
    // Cross product 1: cross(q.xyz, v)
    float cx1 = q->y * v->z - q->z * v->y;
    float cy1 = q->z * v->x - q->x * v->z;
    float cz1 = q->x * v->y - q->y * v->x;
    
    // + q.w * v
    cx1 += q->w * v->x;
    cy1 += q->w * v->y;
    cz1 += q->w * v->z;
    
    // Cross product 2: cross(q.xyz, (cross1 + q.w*v))
    float cx2 = q->y * cz1 - q->z * cy1;
    float cy2 = q->z * cx1 - q->x * cz1;
    float cz2 = q->x * cy1 - q->y * cx1;
    
    // v' = v + 2 * cross2
    result->x = v->x + cx2 * 2.0f;
    result->y = v->y + cy2 * 2.0f;
    result->z = v->z + cz2 * 2.0f;
}

// ============================================================================
// INIT
// ============================================================================
void madgwick_init(madgwick_filter_t* filter, float sample_freq, float beta) {
    if (!filter) return;
    
    // Quaternion identité (pas de rotation)
    filter->q.w = 1.0f;
    filter->q.x = 0.0f;
    filter->q.y = 0.0f;
    filter->q.z = 0.0f;
    
    filter->beta = beta;
    filter->sample_freq = sample_freq;
    filter->last_update = millis();
}

// ============================================================================
// UPDATE (cœur de l'algorithme)
// ============================================================================
void madgwick_update(madgwick_filter_t* filter,
                     float gx, float gy, float gz,
                     float ax, float ay, float az) {
    if (!filter) return;
    
    // Calculer dt réel
    uint32_t now = millis();
    float dt = (now - filter->last_update) / 1000.0f;
    filter->last_update = now;
    
    // Si dt trop petit ou trop grand, utiliser fréquence nominale
    if (dt < 0.001f || dt > 0.1f) {
        dt = 1.0f / filter->sample_freq;
    }
    
    // Raccourcis
    float qw = filter->q.w;
    float qx = filter->q.x;
    float qy = filter->q.y;
    float qz = filter->q.z;
    
    // Normaliser l'accéléromètre
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.0001f) return;  // Éviter division par zéro
    
    float inv_norm = 1.0f / norm;
    ax *= inv_norm;
    ay *= inv_norm;
    az *= inv_norm;
    
    // Gradient descent (correction accéléromètre)
    float s0 = 2.0f * (qx * az - qz * ax);
    float s1 = 2.0f * (qy * az - qw * ax);
    float s2 = 2.0f * (qw * ay + qz * ax);
    float s3 = 2.0f * (qx * ay - qy * ax);
    
    // Normaliser gradient
    norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm > 0.0001f) {
        inv_norm = 1.0f / norm;
        s0 *= inv_norm;
        s1 *= inv_norm;
        s2 *= inv_norm;
        s3 *= inv_norm;
    }
    
    // Intégration gyroscope
    float qDot1 = 0.5f * (-qx * gx - qy * gy - qz * gz);
    float qDot2 = 0.5f * ( qw * gx + qy * gz - qz * gy);
    float qDot3 = 0.5f * ( qw * gy - qx * gz + qz * gx);
    float qDot4 = 0.5f * ( qw * gz + qx * gy - qy * gx);
    
    // Appliquer correction accéléromètre
    qDot1 -= filter->beta * s0;
    qDot2 -= filter->beta * s1;
    qDot3 -= filter->beta * s2;
    qDot4 -= filter->beta * s3;
    
    // Intégrer
    filter->q.w += qDot1 * dt;
    filter->q.x += qDot2 * dt;
    filter->q.y += qDot3 * dt;
    filter->q.z += qDot4 * dt;
    
    // Normaliser quaternion
    quaternion_normalize(&filter->q);
}

// ============================================================================
// CONVERSION QUATERNION → EULER
// ============================================================================
void madgwick_quaternion_to_euler(const quaternion_t* q, euler_t* euler) {
    if (!q || !euler) return;
    
    // Roll (rotation autour X)
    float sinr_cosp = 2.0f * (q->w * q->x + q->y * q->z);
    float cosr_cosp = 1.0f - 2.0f * (q->x * q->x + q->y * q->y);
    euler->roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;
    
    // Pitch (rotation autour Y)
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);
    if (fabsf(sinp) >= 1.0f) {
        euler->pitch = copysignf(90.0f, sinp);  // Gimbal lock
    } else {
        euler->pitch = asinf(sinp) * 180.0f / PI;
    }
    
    // Yaw (rotation autour Z)
    float siny_cosp = 2.0f * (q->w * q->z + q->x * q->y);
    float cosy_cosp = 1.0f - 2.0f * (q->y * q->y + q->z * q->z);
    euler->yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
}

// ============================================================================
// ACCÉLÉRATION RÉFÉRENTIEL TERRESTRE
// ============================================================================
void madgwick_get_earth_accel(const madgwick_filter_t* filter,
                               float ax, float ay, float az,
                               vector3_t* earth_accel) {
    if (!filter || !earth_accel) return;
    
    // Vecteur accélération capteur
    vector3_t sensor_accel = {ax, ay, az};
    
    // Rotation par quaternion inverse (capteur → terre)
    quaternion_t q_inv = filter->q;
    q_inv.x = -q_inv.x;
    q_inv.y = -q_inv.y;
    q_inv.z = -q_inv.z;
    
    quaternion_rotate_vector(&q_inv, &sensor_accel, earth_accel);
}

// ============================================================================
// ACCÉLÉRATION VERTICALE (CRITICAL POUR VARIO)
// ============================================================================
float madgwick_get_vertical_accel(const madgwick_filter_t* filter,
                                   float ax, float ay, float az) {
    if (!filter) return 0.0f;
    
    // Transformer accélération dans référentiel terrestre
    vector3_t earth_accel;
    madgwick_get_earth_accel(filter, ax, ay, az, &earth_accel);
    
    // Soustraire gravité (composante Z)
    // L'accélération verticale = accel_z_earth - gravité
    float vertical_accel = earth_accel.z - GRAVITY_EARTH;
    
    return vertical_accel;
}

// ============================================================================
// RESET
// ============================================================================
void madgwick_reset(madgwick_filter_t* filter) {
    if (!filter) return;
    
    filter->q.w = 1.0f;
    filter->q.x = 0.0f;
    filter->q.y = 0.0f;
    filter->q.z = 0.0f;
    
    filter->last_update = millis();
}