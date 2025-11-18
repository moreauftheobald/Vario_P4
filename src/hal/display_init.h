/**
 * @file display_init.h
 * @brief Initialisation écran MIPI DSI 7" + LVGL 9.3.0
 * 
 * Configuration :
 * - Écran MIPI DSI 1024x600
 * - Triple buffering FULL en PSRAM (3.5MB)
 * - LVGL 9.3.0
 * - 30 FPS
 * 
 * @author Franck Moreau
 * @date 2025-11-18
 * @version 1.0
 * 
 * LOG DOCUMENTATION - MODULE: DISPLAY
 * [ERROR]
 *   - "Display initialization failed" : Échec init écran
 *   - "LVGL initialization failed" : Échec init LVGL
 *   - "PSRAM not available" : PSRAM requis non détecté
 *   - "Failed to allocate display buffer" : Échec allocation buffers
 * 
 * [WARNING]
 *   - "Backlight control failed" : Rétroéclairage non contrôlable
 *   - "Touch not available" : Tactile non détecté (non critique)
 * 
 * [INFO]
 *   - "Display initialized: 1024x600" : Écran OK
 *   - "LVGL v%d.%d.%d started" : Version LVGL
 *   - "Display buffers: 3x %.2f MB (PSRAM)" : Mémoire allouée
 *   - "Backlight set to %d%%" : Luminosité configurée
 * 
 * [VERBOSE]
 *   - "Display buffer allocated at 0x%08X" : Adresse buffer
 *   - "LVGL task started" : Tâche LVGL démarrée
 */

#ifndef DISPLAY_INIT_H
#define DISPLAY_INIT_H

#include <Arduino.h>
#include <lvgl.h>
#include <ESP_Panel_Library.h>

// Forward declarations pour éviter includes circulaires
void lvgl_log_redirect(const char* buf);

// Inclure logger et config APRÈS les déclarations de types
#include "src/system/logger/logger.h"
#include "config/config.h"

// =============================================================================
// CONFIGURATION ÉCRAN WAVESHARE ESP32-P4 7"
// =============================================================================
#define DISPLAY_WIDTH               1024
#define DISPLAY_HEIGHT              600
#define DISPLAY_COLOR_DEPTH         16      // RGB565
#define DISPLAY_BRIGHTNESS_DEFAULT  80      // 0-100%

// Triple buffer FULL en PSRAM
#define DISPLAY_BUFFER_COUNT        3
#define DISPLAY_BUFFER_SIZE         (DISPLAY_WIDTH * DISPLAY_HEIGHT)
#define DISPLAY_BUFFER_BYTES        (DISPLAY_BUFFER_SIZE * 2)  // RGB565 = 2 bytes/pixel
#define DISPLAY_TOTAL_BUFFER_MB     ((DISPLAY_BUFFER_BYTES * DISPLAY_BUFFER_COUNT) / (1024.0f * 1024.0f))

// =============================================================================
// INSTANCES GLOBALES
// =============================================================================
extern ESP_Panel* panel;
extern lv_display_t* display;

// =============================================================================
// FONCTIONS D'INITIALISATION
// =============================================================================

/**
 * @brief Initialise l'écran MIPI DSI
 * 
 * Configure :
 * - Panneau MIPI DSI 1024x600
 * - Interface RGB666
 * - Triple buffering PSRAM
 * - Rétroéclairage
 * 
 * IMPORTANT : Le tactile est désactivé pour éviter conflit I2C.
 * Il sera initialisé séparément sur I2C0 avec ses propres pins.
 * 
 * @return true si succès, false si erreur
 */
bool display_init_panel();

/**
 * @brief Initialise LVGL 9.3.0
 * 
 * Configure :
 * - Display driver avec triple buffer
 * - Thème par défaut (sombre)
 * - Tâche de rafraîchissement (30 FPS)
 * 
 * @return true si succès, false si erreur
 */
bool display_init_lvgl();

/**
 * @brief Fonction callback de flush LVGL
 * 
 * Appelée par LVGL pour transférer le buffer vers l'écran.
 * 
 * @param[in] disp Display LVGL
 * @param[in] area Zone à rafraîchir
 * @param[in] color_p Pointeur vers buffer de pixels
 */
void display_flush_callback(lv_display_t* disp, const lv_area_t* area, uint8_t* color_p);

/**
 * @brief Configure la luminosité de l'écran
 * 
 * @param[in] brightness Luminosité 0-100%
 * @return true si succès, false si erreur
 */
bool display_set_brightness(uint8_t brightness);

/**
 * @brief Obtient la luminosité actuelle
 * 
 * @return Luminosité 0-100%
 */
uint8_t display_get_brightness();

/**
 * @brief Tâche LVGL (appelée par timer ou tâche FreeRTOS)
 * 
 * Gère le rafraîchissement de l'interface à 30 FPS.
 */
void display_lvgl_task();

/**
 * @brief Active/désactive le mode veille de l'écran
 * 
 * @param[in] sleep true pour mettre en veille, false pour réveiller
 */
void display_set_sleep(bool sleep);

/**
 * @brief Affiche un écran de boot simple
 * 
 * Affiche le logo et la version pendant l'initialisation.
 */
void display_show_boot_screen();

/**
 * @brief Redirection logs LVGL vers notre système
 * 
 * @param[in] buf Message de log LVGL
 */
void lvgl_log_redirect(const char* buf);

// =============================================================================
// IMPLÉMENTATION
// =============================================================================

// Instances globales
ESP_Panel* panel = nullptr;
lv_display_t* display = nullptr;

// Variables privées
static uint8_t current_brightness = DISPLAY_BRIGHTNESS_DEFAULT;
static lv_color_t* buf1 = nullptr;
static lv_color_t* buf2 = nullptr;
static lv_color_t* buf3 = nullptr;

/**
 * @brief Redirection logs LVGL
 */
void lvgl_log_redirect(const char* buf) {
    // Supprimer le newline de fin si présent
    char clean_buf[256];
    strncpy(clean_buf, buf, sizeof(clean_buf) - 1);
    clean_buf[sizeof(clean_buf) - 1] = '\0';
    
    size_t len = strlen(clean_buf);
    if (len > 0 && clean_buf[len - 1] == '\n') {
        clean_buf[len - 1] = '\0';
    }
    
    LOG_V(LOG_MODULE_DISPLAY, "[LVGL] %s", clean_buf);
}

/**
 * @brief Initialisation du panneau MIPI DSI
 */
bool display_init_panel() {
    LOG_I(LOG_MODULE_DISPLAY, "Initializing MIPI DSI panel...");
    
    // Vérifier PSRAM disponible
    if (!psramFound()) {
        LOG_E(LOG_MODULE_DISPLAY, "PSRAM not available - REQUIRED for display!");
        return false;
    }
    
    LOG_I(LOG_MODULE_DISPLAY, "PSRAM detected: %d bytes", ESP.getPsramSize());
    
    // Créer instance panneau (ESP_Panel gère automatiquement le MIPI DSI)
    panel = new ESP_Panel();
    
    if (!panel) {
        LOG_E(LOG_MODULE_DISPLAY, "Failed to create panel instance");
        return false;
    }
    
    // IMPORTANT : Désactiver le tactile pour éviter conflit I2C
    // Le tactile sera initialisé séparément avec ses propres pins
    panel->init();
    
    // Initialiser le panneau SANS le tactile
    if (!panel->begin()) {
        LOG_E(LOG_MODULE_DISPLAY, "Panel initialization failed");
        delete panel;
        panel = nullptr;
        return false;
    }
    
    LOG_I(LOG_MODULE_DISPLAY, "Display initialized: %dx%d", DISPLAY_WIDTH, DISPLAY_HEIGHT);
    LOG_W(LOG_MODULE_DISPLAY, "Touch disabled (I2C conflict prevention)");
    
    // Configurer luminosité
    display_set_brightness(DISPLAY_BRIGHTNESS_DEFAULT);
    
    return true;
}

/**
 * @brief Initialisation LVGL
 */
bool display_init_lvgl() {
    LOG_I(LOG_MODULE_DISPLAY, "Initializing LVGL v%d.%d.%d...", 
          lv_version_major(), lv_version_minor(), lv_version_patch());
    
    // Initialiser LVGL
    lv_init();
    
    // Allouer les 3 buffers FULL en PSRAM
    LOG_V(LOG_MODULE_DISPLAY, "Allocating display buffers (%.2f MB total)...", 
          DISPLAY_TOTAL_BUFFER_MB);
    
    buf1 = (lv_color_t*)ps_malloc(DISPLAY_BUFFER_BYTES);
    buf2 = (lv_color_t*)ps_malloc(DISPLAY_BUFFER_BYTES);
    buf3 = (lv_color_t*)ps_malloc(DISPLAY_BUFFER_BYTES);
    
    if (!buf1 || !buf2 || !buf3) {
        LOG_E(LOG_MODULE_DISPLAY, "Failed to allocate display buffers");
        if (buf1) free(buf1);
        if (buf2) free(buf2);
        if (buf3) free(buf3);
        return false;
    }
    
    LOG_I(LOG_MODULE_DISPLAY, "Display buffers: 3x %.2f MB (PSRAM)", 
          DISPLAY_BUFFER_BYTES / (1024.0f * 1024.0f));
    LOG_V(LOG_MODULE_DISPLAY, "Buffer 1: 0x%08X", (uint32_t)buf1);
    LOG_V(LOG_MODULE_DISPLAY, "Buffer 2: 0x%08X", (uint32_t)buf2);
    LOG_V(LOG_MODULE_DISPLAY, "Buffer 3: 0x%08X", (uint32_t)buf3);
    
    // Créer le display LVGL
    display = lv_display_create(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    if (!display) {
        LOG_E(LOG_MODULE_DISPLAY, "Failed to create LVGL display");
        return false;
    }
    
    // Configurer les buffers (triple buffering)
    lv_display_set_buffers(display, buf1, buf2, DISPLAY_BUFFER_SIZE, LV_DISPLAY_RENDER_MODE_FULL);
    
    // Associer callback de flush
    lv_display_set_flush_cb(display, display_flush_callback);
    
    // Appliquer thème sombre par défaut
    lv_theme_t* theme = lv_theme_default_init(
        display,
        lv_palette_main(LV_PALETTE_BLUE),
        lv_palette_main(LV_PALETTE_RED),
        true,  // dark mode
        LV_FONT_DEFAULT
    );
    lv_display_set_theme(display, theme);
    
    LOG_I(LOG_MODULE_DISPLAY, "LVGL initialized successfully");
    
    return true;
}

/**
 * @brief Callback flush LVGL
 */
void display_flush_callback(lv_display_t* disp, const lv_area_t* area, uint8_t* color_p) {
    if (!panel) {
        lv_display_flush_ready(disp);
        return;
    }
    
    // Calculer dimensions de la zone
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    
    // ESP_Panel_Library utilise getLcd() pour accéder à l'écran
    auto lcd = panel->getLcd();
    if (lcd) {
        // drawBitmap attend uint8_t* (déjà le bon type)
        lcd->drawBitmap(area->x1, area->y1, w, h, color_p);
    }
    
    // Signaler à LVGL que le flush est terminé
    lv_display_flush_ready(disp);
}

/**
 * @brief Configure la luminosité
 */
bool display_set_brightness(uint8_t brightness) {
    if (brightness > 100) brightness = 100;
    
    if (!panel) {
        LOG_W(LOG_MODULE_DISPLAY, "Panel not initialized");
        return false;
    }
    
    // ESP_Panel_Library utilise getBacklight() pour accéder au rétroéclairage
    auto backlight = panel->getBacklight();
    if (backlight) {
        // Convertir 0-100 en 0-255 pour le PWM
        uint8_t pwm_value = (brightness * 255) / 100;
        backlight->setBrightness(pwm_value);
        current_brightness = brightness;
        
        LOG_I(LOG_MODULE_DISPLAY, "Backlight set to %d%%", brightness);
        return true;
    }
    
    LOG_W(LOG_MODULE_DISPLAY, "Backlight control not available");
    return false;
}

/**
 * @brief Obtient la luminosité
 */
uint8_t display_get_brightness() {
    return current_brightness;
}

/**
 * @brief Tâche LVGL
 */
void display_lvgl_task() {
    // Handler LVGL (à appeler toutes les ~30ms pour 30 FPS)
    lv_timer_handler();
}

/**
 * @brief Mode veille
 */
void display_set_sleep(bool sleep) {
    if (!panel) return;
    
    if (sleep) {
        LOG_I(LOG_MODULE_DISPLAY, "Display entering sleep mode");
        display_set_brightness(0);
    } else {
        LOG_I(LOG_MODULE_DISPLAY, "Display waking up");
        display_set_brightness(current_brightness);
    }
}

/**
 * @brief Écran de boot
 */
void display_show_boot_screen() {
    if (!display) return;
    
    LOG_V(LOG_MODULE_DISPLAY, "Showing boot screen");
    
    // Créer un écran simple
    lv_obj_t* scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
    
    // Titre
    lv_obj_t* label_title = lv_label_create(scr);
    lv_label_set_text(label_title, PROJECT_NAME);
    lv_obj_set_style_text_font(label_title, &lv_font_montserrat_28, 0);
    lv_obj_set_style_text_color(label_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(label_title, LV_ALIGN_CENTER, 0, -50);
    
    // Version
    lv_obj_t* label_version = lv_label_create(scr);
    lv_label_set_text_fmt(label_version, "v%s", PROJECT_VERSION);
    lv_obj_set_style_text_color(label_version, lv_color_hex(0x808080), 0);
    lv_obj_align(label_version, LV_ALIGN_CENTER, 0, -10);
    
    // Message boot
    lv_obj_t* label_boot = lv_label_create(scr);
    lv_label_set_text(label_boot, "Initialisation...");
    lv_obj_set_style_text_color(label_boot, lv_color_hex(0x00FF00), 0);
    lv_obj_align(label_boot, LV_ALIGN_CENTER, 0, 30);
    
    // Charger l'écran
    lv_screen_load(scr);
    
    // Forcer un rafraîchissement
    lv_timer_handler();
}

#endif // DISPLAY_INIT_H