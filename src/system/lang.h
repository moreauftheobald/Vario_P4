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
const char* trad[][50] = {
  // FRANÇAIS
  {
    "Variometre ESP32-P4",           // 0
    "Touchez l'ecran!",              // 1
    "TEST",                          // 2
    "Pret",                          // 3
    "Altitude",                      // 4
    "Vario",                         // 5
    "Vitesse",                       // 6
    "Finesse",                       // 7
    "Distance",                      // 8
    "Cap",                           // 9
    "Satellites",                    // 10
    
    // Écran préstart
    "Info Systeme",                  // 11
    "Version",                       // 12
    "SD",                            // 13
    "Barometre",                     // 14
    "IMU",                           // 15
    "GPS",                           // 16
    "WiFi",                          // 17
    "Batterie",                      // 18
    "Kalman",                        // 19
    
    // Info pilote
    "Pilote",                        // 20
    "Nom",                           // 21
    "Prenom",                        // 22
    "Telephone",                     // 23
    "Voile",                         // 24
    
    // Contact ICE
    "Contact Urgence (ICE)",         // 25
    
    // Boutons
    "Demarrage",                     // 26
    "Parametres",                    // 27
    "Transfert",                     // 28
    
    // États
    "OK",                            // 29
    "ERROR",                         // 30
    "NO FIX",                        // 31
    "FIX",                           // 32
    "INIT...",                       // 33
    "Connexion...",                  // 34
    "libre",                         // 35
    "sats",                          // 36
    
    // Unités
    "hPa",                           // 37
    "m",                             // 38
    "km/h",                          // 39
    "m/s",                           // 40
    
    // Messages
    "Non disponible",                // 41
    "Chargement...",                 // 42
    "Pret au decollage",             // 43
  }
};

#endif  // LANG_H