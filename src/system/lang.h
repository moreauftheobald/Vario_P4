/**
 * @file lang.h
 * @brief Système de traduction simple
 * 
 * Usage: trad[current_lang][0] pour accéder aux textes
 */

#ifndef LANG_H
#define LANG_H

#include <Arduino.h>

// =============================================================================
// LANGUES
// =============================================================================
enum Language {
  LANG_FR = 0,
  LANG_EN = 1,
  LANG_COUNT
};

// Langue active
static Language current_lang = LANG_FR;

// =============================================================================
// TRADUCTIONS (pour l'instant seulement français)
// =============================================================================
const char* trad[][11] = {
  // FRANÇAIS
  {
    "Variometre ESP32-P4",      // 0
    "Touchez l'ecran!",         // 1
    "TEST",                     // 2
    "Pret",                     // 3
    "Altitude",                 // 4
    "Vario",                    // 5
    "Vitesse",                  // 6
    "Finesse",                  // 7
    "Distance",                 // 8
    "Cap",                      // 9
    "Satellites",               // 10
  }
};

#endif  // LANG_H