/**
 * @file splash_screen.h
 * @brief Splash screen avec logo BipBipHourra
 */

#ifndef SPLASH_SCREEN_H
#define SPLASH_SCREEN_H

#include <Arduino.h>
#include <lvgl.h>
#include "src/system/logger.h"


// Déclaration externe du logo (défini dans logo_bipbiphourra.c)
LV_IMG_DECLARE(logo_bipbiphourra);

// =============================================================================
// CONFIGURATION
// =============================================================================

#define SPLASH_BG_COLOR 0xFFF7E6  // Beige/Crème

// =============================================================================
// VARIABLES GLOBALES
// =============================================================================

static lv_obj_t* splash_screen = NULL;
static uint32_t splash_start_time = 0;
static uint32_t splash_duration = 0;

// =============================================================================
// FONCTIONS
// =============================================================================

/**
 * @brief Crée et affiche le splash screen
 * @param duration_ms Durée d'affichage en ms
 */
void splash_screen_create(uint32_t duration_ms) {
  LOG_I(LOG_UI, "Creating splash screen (%lu ms)", duration_ms);
  
  // Créer l'écran
  splash_screen = lv_obj_create(NULL);
  
  // Fond beige/crème
  lv_obj_set_style_bg_color(splash_screen, lv_color_hex(SPLASH_BG_COLOR), 0);
  
  // Créer l'image du logo
  lv_obj_t* logo = lv_img_create(splash_screen);
  lv_img_set_src(logo, &logo_bipbiphourra);
  
  // Centrer le logo (500x500 sur écran 1024x600)
  lv_obj_align(logo, LV_ALIGN_CENTER, 0, 0);
  
  // Charger l'écran
  lv_screen_load(splash_screen);
  
  // Forcer le rafraîchissement immédiat
  lv_refr_now(display);
  
  // Enregistrer le timing
  splash_start_time = millis();
  splash_duration = duration_ms;
  
  LOG_I(LOG_UI, "Splash screen displayed");
}

/**
 * @brief Vérifie si le splash screen doit être fermé
 * @return true si le splash doit être fermé
 */
bool splash_screen_should_close() {
  if (splash_screen == NULL) return false;
  return (millis() - splash_start_time >= splash_duration);
}

/**
 * @brief Ferme le splash screen
 */
void splash_screen_close() {
  if (splash_screen == NULL) return;
  
  LOG_I(LOG_UI, "Closing splash screen");
  
  // Supprimer l'écran
  lv_obj_del(splash_screen);
  splash_screen = NULL;
  
  LOG_I(LOG_UI, "Splash screen closed");
}

/**
 * @brief Affiche le splash screen de manière bloquante
 * @param duration_ms Durée d'affichage en ms
 */
void splash_screen_show_blocking(uint32_t duration_ms) {
  splash_screen_create(duration_ms);
  
  unsigned long start = millis();
  while (millis() - start < duration_ms) {
    lv_timer_handler();
    delay(5);
  }
  
  splash_screen_close();
}

/**
 * @brief Affiche le splash screen de manière non-bloquante
 * À appeler dans setup(), puis splash_screen_close() quand prêt
 */
void splash_screen_show(uint32_t duration_ms) {
  splash_screen_create(duration_ms);
  // L'appelant doit appeler splash_screen_close() manuellement
}

#endif  // SPLASH_SCREEN_H