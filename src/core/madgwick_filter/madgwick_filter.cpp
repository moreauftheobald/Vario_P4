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
 * 
 * Utilise la formule optimisée sans calcul matriciel.
 */
static void quaternion_rotate_vector(const quaternion_t* q, const vector3_t* v, vector3_t* result) {
    // Formule: v' = v + 2*q.w*cross(q.xyz, v) + 2*cross(q.xyz, cross(q.xyz, v))
    
    // Étape 1: Calculer cross(q.xyz, v)
    float cx = q->y * v->z - q->z * v->y;
    float cy = q->z * v->x - q->x * v->z;
    float cz = q->x * v->y - q->y * v->x;
    
    // Étape 2: Calculer cross(q.xyz, cross(q.xyz, v))
    float ccx = q->y * cz - q->z * cy;
    float ccy = q->z * cx - q->x * cz;
    float ccz = q->x * cy - q->y * cx;
    
    // Étape 3: v' = v + 2*q.w*cross(q.xyz, v) + 2*cross(q.xyz, cross(q.xyz, v))
    result->x = v->x + 2.0f * (q->w * cx + ccx);
    result->y = v->y + 2.0f * (q->w * cy + ccy);
    result->z = v->z + 2.0f * (q->w * cz + ccz);
}

// ============================================================================
// INIT
// ============================================================================

/**
 * @brief Initialise le quaternion depuis l'accéléromètre
 * 
 * Estime l'orientation initiale en supposant que l'accéléromètre
 * mesure uniquement la gravité (pas de mouvement).
 */
void madgwick_init_from_accel(madgwick_filter_t* filter, 
                               float ax, float ay, float az) {
    if (!filter) return;
    
    // Normaliser l'accéléromètre
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.0001f) return;
    
    ax /= norm;
    ay /= norm;
    az /= norm;
    
    // Calculer roll et pitch depuis l'accéléromètre
    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    
    // Convertir en quaternion (yaw = 0)
    float cy = cosf(0.0f * 0.5f);
    float sy = sinf(0.0f * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    
    filter->q.w = cr * cp * cy + sr * sp * sy;
    filter->q.x = sr * cp * cy - cr * sp * sy;
    filter->q.y = cr * sp * cy + sr * cp * sy;
    filter->q.z = cr * cp * sy - sr * sp * cy;
    
    quaternion_normalize(&filter->q);
}

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
    
    quaternion_rotate_vector(&filter->q, &sensor_accel, earth_accel);
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

// ============================================================================
// TEST / DEBUG
// ============================================================================
void madgwick_rotate_vector_test(const quaternion_t* q, 
                                  float vx, float vy, float vz,
                                  float* rx, float* ry, float* rz) {
    if (!q || !rx || !ry || !rz) return;
    
    vector3_t v = {vx, vy, vz};
    vector3_t result;
    
    quaternion_rotate_vector(q, &v, &result);
    
    *rx = result.x;
    *ry = result.y;
    *rz = result.z;
}