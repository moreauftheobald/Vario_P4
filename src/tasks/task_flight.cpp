/**
 * @file task_flight.cpp
 * @brief Implémentation tâche flight unifiée
 * 
 * @author Franck Moreau
 * @date 2025-11-15
 * @version 1.0
 */

#include "src/tasks/task_flight.h"
#include "src/system/sensor_init/sensor_init.h"
#include "src/system/imu_calibration/imu_calibration.h"
#include "src/system/logger/logger.h"
#include "config/config.h"
#include <Adafruit_AHRS.h>
#include <math.h>

// === Instances globales ===
Adafruit_Madgwick filter_madgwick;
SemaphoreHandle_t flight_data_mutex = NULL;
flight_data_t g_flight_data = {0};

// === Variables privées tâche ===
static TaskHandle_t task_handle = NULL;

// Compteurs diviseurs de fréquence
static uint16_t counter_main = 0;

// Données capteurs brutes
static float gyro_x, gyro_y, gyro_z;
static float accel_x, accel_y, accel_z;
static float pressure, temperature;
static float gps_lat, gps_lon, gps_alt, gps_speed;

// État Kalman (2 états : altitude, vitesse_verticale)
static float kalman_altitude = 0;
static float kalman_vario = 0;
static float kalman_P[2][2] = {{1, 0}, {0, 1}};  // Matrice covariance

// Historique pour moyennes
static float vario_history[VARIO_HISTORY_SIZE] = {0};
static uint8_t vario_history_index = 0;

// Références décollage
static bool takeoff_detected = false;
static float takeoff_altitude = 0;
static float takeoff_latitude = 0;
static float takeoff_longitude = 0;

/**
 * @brief Initialise la tâche flight
 */
bool task_flight_init() {
    LOG_I(LOG_MODULE_FLIGHT, "Initializing flight task...");
    
    // Créer mutex
    flight_data_mutex = xSemaphoreCreateMutex();
    if (flight_data_mutex == NULL) {
        LOG_E(LOG_MODULE_FLIGHT, "Flight data mutex creation failed");
        return false;
    }
    
    // Initialiser Madgwick
    filter_madgwick.begin(MADGWICK_SAMPLE_FREQ);
    LOG_V(LOG_MODULE_FLIGHT, "Madgwick filter initialized @ %d Hz", MADGWICK_SAMPLE_FREQ);
    
    // Initialiser Kalman
    task_flight_kalman_init();
    
    // Initialiser flight_data
    memset(&g_flight_data, 0, sizeof(flight_data_t));
    g_flight_data.qnh = QNH_STANDARD;
    
    // Créer tâche FreeRTOS
    BaseType_t result = xTaskCreatePinnedToCore(
        task_flight_run,
        "task_flight",
        TASK_SENSORS_STACK_SIZE,
        NULL,
        TASK_SENSORS_PRIORITY,
        &task_handle,
        TASK_SENSORS_CORE
    );
    
    if (result != pdPASS) {
        LOG_E(LOG_MODULE_FLIGHT, "Task flight creation failed");
        return false;
    }
    
    LOG_I(LOG_MODULE_FLIGHT, "Flight task started @ %d Hz", TASK_SENSORS_FREQ_HZ);
    return true;
}

/**
 * @brief Boucle principale tâche flight
 */
void task_flight_run(void* params) {
    TickType_t last_wake = xTaskGetTickCount();
    
    LOG_I(LOG_MODULE_FLIGHT, "Flight task running");
    
    // Variables pour monitoring performance
    uint32_t overrun_count = 0;
    uint32_t last_overrun_log = 0;
    
    while (1) {
        uint32_t loop_start = micros();  // Utiliser micros() pour précision
        
        // 1. Lecture capteurs (200 Hz)
        task_flight_read_sensors();
        
        // 2. Fusion Madgwick (200 Hz)
        task_flight_fusion_madgwick();
        
        // 3. Kalman filter (100 Hz, diviseur /2)
        if (counter_main % FREQ_KALMAN_DIVIDER == 0) {
            task_flight_kalman_filter();
        }
        
        // 4. Calculs paramètres vol (100 Hz)
        if (counter_main % FREQ_KALMAN_DIVIDER == 0) {
            task_flight_compute_parameters();
        }
        
        // 5. Stockage dans flight_data
        task_flight_store_data();
        
        // Incrémenter compteur
        counter_main++;
        if (counter_main >= 200) counter_main = 0;
        
        // Vérifier timing (ne logger que toutes les 10 secondes si overrun)
        uint32_t loop_time = (micros() - loop_start) / 1000;  // Convertir en ms
        if (loop_time > TASK_SENSORS_PERIOD_MS) {
            overrun_count++;
            if (millis() - last_overrun_log > 10000) {  // Log tous les 10s max
                LOG_W(LOG_MODULE_FLIGHT, "Flight task overrun: %d ms (%d overruns)", 
                      loop_time, overrun_count);
                last_overrun_log = millis();
                overrun_count = 0;
            }
        }
        
        // Attendre prochaine période
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_SENSORS_PERIOD_MS));
    }
}

/**
 * @brief Lecture capteurs
 */
void task_flight_read_sensors() {
    // === LSM6DSO32 (200 Hz) ===
    if (sensor_lsm6dso32_ready) {
        sensors_event_t accel, gyro, temp;
        if (lsm6dso32.getEvent(&accel, &gyro, &temp)) {
            // Récupérer données brutes
            gyro_x = gyro.gyro.x;
            gyro_y = gyro.gyro.y;
            gyro_z = gyro.gyro.z;
            
            accel_x = accel.acceleration.x;
            accel_y = accel.acceleration.y;
            accel_z = accel.acceleration.z;
            
            temperature = temp.temperature;
            
            // Appliquer calibration
            imu_calibration_apply(&gyro_x, &gyro_y, &gyro_z,
                                 &accel_x, &accel_y, &accel_z);
        } else {
            LOG_E(LOG_MODULE_FLIGHT, "LSM6DSO32 read error");
        }
    }
    
    // === BMP390 (50 Hz, diviseur /4) ===
    if (sensor_bmp390_ready && (counter_main % FREQ_BMP390_DIVIDER == 0)) {
        if (bmp390.performReading()) {
            pressure = bmp390.pressure / 100.0f;  // Convertir en hPa
            // température déjà lue depuis LSM6DSO32
        } else {
            LOG_E(LOG_MODULE_FLIGHT, "BMP390 read error");
        }
    }
    
    // === GPS (2 Hz, diviseur /100) ===
    if (sensor_gps_ready && (counter_main % FREQ_GPS_DIVIDER == 0)) {
        // Lire GPS de manière non-bloquante
        // Ne pas appeler gps.read() en boucle, juste vérifier une fois
        if (gps.available()) {
            char c = gps.read();
            if (gps.newNMEAreceived()) {
                if (gps.parse(gps.lastNMEA())) {
                    if (gps.fix) {
                        gps_lat = gps.latitudeDegrees;
                        gps_lon = gps.longitudeDegrees;
                        gps_alt = gps.altitude;
                        gps_speed = gps.speed * 1.852f;  // Convertir knots -> km/h
                    }
                }
            }
        }
    }
}

/**
 * @brief Fusion Madgwick
 */
void task_flight_fusion_madgwick() {
    // Update Madgwick avec magnéto à zéro (6-axis seulement)
    // API Adafruit_AHRS : update(gx, gy, gz, ax, ay, az, mx, my, mz)
    filter_madgwick.update(
        gyro_x, gyro_y, gyro_z,
        accel_x, accel_y, accel_z,
        0, 0, 0  // Pas de magnétomètre
    );
    
    // Les quaternions sont maintenant à jour dans filter_madgwick
}

/**
 * @brief Extrait accélération verticale
 */
float task_flight_get_vertical_acceleration() {
    // Récupérer quaternions
    // API Adafruit_AHRS : getQuaternion(&qw, &qx, &qy, &qz)
    float qw, qx, qy, qz;
    filter_madgwick.getQuaternion(&qw, &qx, &qy, &qz);
    
    // Rotation inverse pour référentiel terrestre
    // Formule rotation par quaternion : v' = q * v * q^-1
    float ax_earth = 2*(qx*qz - qw*qy)*accel_x + 
                     2*(qy*qz + qw*qx)*accel_y + 
                     (qw*qw - qx*qx - qy*qy + qz*qz)*accel_z;
    
    // Retirer gravité pour avoir accélération pure
    float az_vertical = ax_earth - GRAVITY_STANDARD;
    
    return az_vertical;
}

/**
 * @brief Filtre de Kalman
 */
void task_flight_kalman_filter() {
    float dt = 1.0f / FREQ_KALMAN_HZ;  // 0.01 s
    
    // === PRÉDICTION ===
    // État: [altitude, vario]
    // Modèle: altitude(k+1) = altitude(k) + vario(k) * dt
    //         vario(k+1) = vario(k) + accel_vertical * dt
    
    float accel_vert = task_flight_get_vertical_acceleration();
    
    // Prédiction état
    float altitude_pred = kalman_altitude + kalman_vario * dt;
    float vario_pred = kalman_vario + accel_vert * dt;
    
    // Prédiction covariance
    // P = F * P * F' + Q
    float F[2][2] = {{1, dt}, {0, 1}};
    float Q[2][2] = {{KALMAN_PROCESS_NOISE_ALT, 0}, 
                     {0, KALMAN_PROCESS_NOISE_VARIO}};
    
    float P_temp[2][2];
    // P_temp = F * P
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            P_temp[i][j] = 0;
            for (int k = 0; k < 2; k++) {
                P_temp[i][j] += F[i][k] * kalman_P[k][j];
            }
        }
    }
    
    // P = P_temp * F' + Q
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            kalman_P[i][j] = Q[i][j];
            for (int k = 0; k < 2; k++) {
                kalman_P[i][j] += P_temp[i][k] * F[j][k];  // F' = F transposé
            }
        }
    }
    
    // === MISE À JOUR (mesure baromètre) ===
    // Calcul altitude depuis pression
    float altitude_baro = 44330.0f * (1.0f - pow(pressure / g_flight_data.qnh, 0.1903f));
    
    // Innovation (différence mesure - prédiction)
    float innovation = altitude_baro - altitude_pred;
    
    // Gain de Kalman
    // K = P * H' / (H * P * H' + R)
    // H = [1 0] (on mesure l'altitude seulement)
    float S = kalman_P[0][0] + KALMAN_MEASURE_NOISE_BARO;
    float K[2] = {kalman_P[0][0] / S, kalman_P[1][0] / S};
    
    // Mise à jour état
    kalman_altitude = altitude_pred + K[0] * innovation;
    kalman_vario = vario_pred + K[1] * innovation;
    
    // Mise à jour covariance
    // P = (I - K * H) * P
    float I_KH[2][2] = {{1 - K[0], 0}, {-K[1], 1}};
    float P_new[2][2];
    
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            P_new[i][j] = 0;
            for (int k = 0; k < 2; k++) {
                P_new[i][j] += I_KH[i][k] * kalman_P[k][j];
            }
        }
    }
    
    // Copier P_new dans kalman_P
    memcpy(kalman_P, P_new, sizeof(kalman_P));
    
    // Stocker dans historique pour moyennes
    vario_history[vario_history_index] = kalman_vario;
    vario_history_index = (vario_history_index + 1) % VARIO_HISTORY_SIZE;
}

/**
 * @brief Calculs paramètres vol
 */
void task_flight_compute_parameters() {
    // === Altitudes ===
    g_flight_data.altitude_qnh = kalman_altitude;
    g_flight_data.altitude_qne = 44330.0f * (1.0f - pow(pressure / QNH_STANDARD, 0.1903f));
    g_flight_data.altitude_gps = gps_alt;
    
    if (takeoff_detected) {
        g_flight_data.altitude_agl = kalman_altitude - takeoff_altitude;
    } else {
        g_flight_data.altitude_agl = 0;
    }
    
    // === Vario ===
    g_flight_data.vario = kalman_vario;
    
    // Vario moyenné 10s (10Hz * 10s = 100 échantillons, mais on a 30)
    float vario_sum = 0;
    for (int i = 0; i < VARIO_HISTORY_SIZE; i++) {
        vario_sum += vario_history[i];
    }
    g_flight_data.vario_avg_10s = vario_sum / VARIO_HISTORY_SIZE;
    
    // === GPS ===
    g_flight_data.latitude = gps_lat;
    g_flight_data.longitude = gps_lon;
    g_flight_data.speed_ground = gps_speed;
    g_flight_data.gps_fix = gps.fix;
    g_flight_data.satellites = gps.satellites;
    
    // === G-Force ===
    float accel_total = sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    g_flight_data.g_force = accel_total / GRAVITY_STANDARD;
    
    // === Temps ===
    if (takeoff_detected) {
        g_flight_data.time_flight = (millis() - g_flight_data.takeoff_time) / 1000;
    }
    
    // === Détection décollage ===
    if (!takeoff_detected && g_flight_data.speed_ground > 5.0f && kalman_vario > 0.5f) {
        takeoff_detected = true;
        takeoff_altitude = kalman_altitude;
        takeoff_latitude = gps_lat;
        takeoff_longitude = gps_lon;
        g_flight_data.takeoff_time = millis();
        g_flight_data.in_flight = true;
        LOG_I(LOG_MODULE_FLIGHT, "Takeoff detected!");
    }
}

/**
 * @brief Stocke données dans flight_data
 */
void task_flight_store_data() {
    if (xSemaphoreTake(flight_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Copier les données calculées
        g_flight_data.timestamp_ms = millis();
        g_flight_data.pressure = pressure;
        g_flight_data.temperature = temperature;
        g_flight_data.accel_vertical = task_flight_get_vertical_acceleration();
        
        // Quaternions
        filter_madgwick.getQuaternion(&g_flight_data.quaternion.w,
                                      &g_flight_data.quaternion.x,
                                      &g_flight_data.quaternion.y,
                                      &g_flight_data.quaternion.z);
        
        // Angles Euler (calculés manuellement depuis quaternions)
        float qw = g_flight_data.quaternion.w;
        float qx = g_flight_data.quaternion.x;
        float qy = g_flight_data.quaternion.y;
        float qz = g_flight_data.quaternion.z;
        
        // Roll (rotation autour X)
        float sinr_cosp = 2 * (qw * qx + qy * qz);
        float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        g_flight_data.roll = atan2(sinr_cosp, cosr_cosp) * 180.0f / PI;
        
        // Pitch (rotation autour Y)
        float sinp = 2 * (qw * qy - qz * qx);
        if (abs(sinp) >= 1)
            g_flight_data.pitch = copysign(90.0f, sinp); // Gimbal lock
        else
            g_flight_data.pitch = asin(sinp) * 180.0f / PI;
        
        // Yaw (rotation autour Z)
        float siny_cosp = 2 * (qw * qz + qx * qy);
        float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        g_flight_data.yaw = atan2(siny_cosp, cosy_cosp) * 180.0f / PI;
        
        xSemaphoreGive(flight_data_mutex);
    }
}

/**
 * @brief Obtient copie thread-safe flight_data
 */
bool task_flight_get_data(flight_data_t* data) {
    if (xSemaphoreTake(flight_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(data, &g_flight_data, sizeof(flight_data_t));
        xSemaphoreGive(flight_data_mutex);
        return true;
    }
    return false;
}

/**
 * @brief Initialise Kalman
 */
void task_flight_kalman_init() {
    kalman_altitude = 0;
    kalman_vario = 0;
    
    // Matrice covariance initiale
    kalman_P[0][0] = 100.0f;  // Incertitude altitude initiale
    kalman_P[0][1] = 0;
    kalman_P[1][0] = 0;
    kalman_P[1][1] = 10.0f;   // Incertitude vario initiale
    
    LOG_V(LOG_MODULE_FLIGHT, "Kalman filter initialized");
}

/**
 * @brief Reset calculs vol
 */
void task_flight_reset() {
    takeoff_detected = false;
    takeoff_altitude = 0;
    
    memset(vario_history, 0, sizeof(vario_history));
    vario_history_index = 0;
    
    // Reset stats
    g_flight_data.altitude_max = 0;
    g_flight_data.altitude_min = 0;
    g_flight_data.distance_total = 0;
    g_flight_data.time_flight = 0;
    
    LOG_I(LOG_MODULE_FLIGHT, "Flight data reset");
}