/**
 * @file ui_config.h
 * @brief Configuration UI - Couleurs, polices, dimensions (Design moderne)
 */

#ifndef UI_CONFIG_H
#define UI_CONFIG_H

#include <lvgl.h>

// =============================================================================
// COULEURS - PALETTE PRINCIPALE (Thème sombre moderne)
// =============================================================================
#define UI_COLOR_BG                 0x0A0A0A  // Noir profond
#define UI_COLOR_BG_SECONDARY       0x1A1A1A  // Gris très foncé
#define UI_COLOR_PANEL              0x1E1E2E  // Bleu-gris foncé
#define UI_COLOR_PANEL_HOVER        0x252535  // Bleu-gris moyen
#define UI_COLOR_PANEL_BORDER       0x00D9FF  // Cyan électrique
#define UI_COLOR_ACCENT_PRIMARY     0x00D9FF  // Cyan électrique
#define UI_COLOR_ACCENT_SECONDARY   0x7B68EE  // Violet médium
#define UI_COLOR_TEXT_PRIMARY       0xFFFFFF  // Blanc pur
#define UI_COLOR_TEXT_SECONDARY     0xB0B0C0  // Gris bleuté
#define UI_COLOR_TEXT_DISABLED      0x505060  // Gris moyen

// =============================================================================
// COULEURS - ÉTATS (Plus vives et contrastées)
// =============================================================================
#define UI_COLOR_SUCCESS            0x00FF88  // Vert néon
#define UI_COLOR_SUCCESS_DARK       0x00AA55  // Vert foncé
#define UI_COLOR_WARNING            0xFFAA00  // Orange vif
#define UI_COLOR_WARNING_DARK       0xCC7700  // Orange foncé
#define UI_COLOR_ERROR              0xFF3366  // Rouge/rose néon
#define UI_COLOR_ERROR_DARK         0xCC0033  // Rouge foncé
#define UI_COLOR_INFO               0x0099FF  // Bleu ciel
#define UI_COLOR_INFO_DARK          0x0066CC  // Bleu foncé

// =============================================================================
// COULEURS - BOUTONS (Gradients et effets)
// =============================================================================
#define UI_COLOR_BTN_START          0x00CC66  // Vert
#define UI_COLOR_BTN_START_GRAD     0x00FF88  // Vert clair (gradient)
#define UI_COLOR_BTN_SETTINGS       0x0088FF  // Bleu
#define UI_COLOR_BTN_SETTINGS_GRAD  0x00AAFF  // Bleu clair
#define UI_COLOR_BTN_FILES          0xFF8800  // Orange
#define UI_COLOR_BTN_FILES_GRAD     0xFFAA33  // Orange clair
#define UI_COLOR_BTN_CANCEL         0x666677  // Gris
#define UI_COLOR_BTN_CONFIRM        0x00CC66  // Vert
#define UI_COLOR_BTN_DELETE         0xFF3366  // Rouge/rose
#define UI_COLOR_BTN_PRESSED        0x2A2A3A  // Gris bleuté foncé

// =============================================================================
// COULEURS - VARIO
// =============================================================================
#define UI_COLOR_VARIO_UP           0x00FF88  // Vert néon (montée)
#define UI_COLOR_VARIO_DOWN         0xFF3366  // Rouge/rose néon (descente)
#define UI_COLOR_VARIO_NEUTRAL      0xFFFFFF  // Blanc (neutre)

// =============================================================================
// COULEURS - CARTE
// =============================================================================
#define UI_COLOR_MAP_TRAIL          0x00D9FF  // Cyan
#define UI_COLOR_MAP_WAYPOINT       0xFF00FF  // Magenta
#define UI_COLOR_MAP_TAKEOFF        0xFFDD00  // Jaune

// =============================================================================
// COULEURS - BADGES DE STATUT
// =============================================================================
#define UI_COLOR_BADGE_ONLINE       0x00FF88  // Vert
#define UI_COLOR_BADGE_OFFLINE      0xFF3366  // Rouge
#define UI_COLOR_BADGE_PENDING      0xFFAA00  // Orange
#define UI_COLOR_BADGE_NEUTRAL      0x666677  // Gris

// =============================================================================
// POLICES
// =============================================================================
#define UI_FONT_TINY                &lv_font_montserrat_12
#define UI_FONT_SMALL               &lv_font_montserrat_14
#define UI_FONT_NORMAL              &lv_font_montserrat_16
#define UI_FONT_MEDIUM              &lv_font_montserrat_20
#define UI_FONT_LARGE               &lv_font_montserrat_24
#define UI_FONT_XLARGE              &lv_font_montserrat_28
#define UI_FONT_HUGE                &lv_font_montserrat_32
#define UI_FONT_VARIO               &lv_font_montserrat_48  // Pour affichage vario

// =============================================================================
// DIMENSIONS - ÉCRAN
// =============================================================================
#define UI_SCREEN_WIDTH             1024
#define UI_SCREEN_HEIGHT            600

// =============================================================================
// DIMENSIONS - MARGES ET ESPACEMENTS
// =============================================================================
#define UI_PAD_NONE                 0
#define UI_PAD_TINY                 7
#define UI_PAD_SMALL                8
#define UI_PAD_MEDIUM               12
#define UI_PAD_LARGE                16
#define UI_PAD_XLARGE               24

#define UI_GAP_TINY                 4
#define UI_GAP_SMALL                8
#define UI_GAP_MEDIUM               12
#define UI_GAP_LARGE                16
#define UI_GAP_XLARGE               24

// =============================================================================
// DIMENSIONS - BORDURES
// =============================================================================
#define UI_BORDER_NONE              0
#define UI_BORDER_THIN              1
#define UI_BORDER_MEDIUM            2
#define UI_BORDER_THICK             3
#define UI_BORDER_BOLD              4

// =============================================================================
// DIMENSIONS - RADIUS (ARRONDIS)
// =============================================================================
#define UI_RADIUS_NONE              0
#define UI_RADIUS_SMALL             8
#define UI_RADIUS_MEDIUM            12
#define UI_RADIUS_LARGE             16
#define UI_RADIUS_XLARGE            20
#define UI_RADIUS_ROUND             50  // Cercle

// =============================================================================
// DIMENSIONS - OMBRES
// =============================================================================
#define UI_SHADOW_WIDTH_SMALL       10
#define UI_SHADOW_WIDTH_MEDIUM      15
#define UI_SHADOW_WIDTH_LARGE       20
#define UI_SHADOW_OFS_X             0
#define UI_SHADOW_OFS_Y             4
#define UI_SHADOW_SPREAD            2
#define UI_SHADOW_COLOR             0x000000
#define UI_SHADOW_OPA_LIGHT         LV_OPA_20
#define UI_SHADOW_OPA_MEDIUM        LV_OPA_40
#define UI_SHADOW_OPA_HEAVY         LV_OPA_60

// =============================================================================
// DIMENSIONS - BOUTONS
// =============================================================================
#define UI_BTN_HEIGHT_SMALL         50
#define UI_BTN_HEIGHT_MEDIUM        70
#define UI_BTN_HEIGHT_LARGE         90

#define UI_BTN_WIDTH_SMALL          120
#define UI_BTN_WIDTH_MEDIUM         220
#define UI_BTN_WIDTH_LARGE          300

// Boutons écran préstart
#define UI_BTN_PRESTART_WIDTH       280
#define UI_BTN_PRESTART_HEIGHT      85

// Boutons écran principal
#define UI_BTN_MAIN_WIDTH           140
#define UI_BTN_MAIN_HEIGHT          70

// =============================================================================
// DIMENSIONS - PANNEAUX ET CARTES
// =============================================================================
#define UI_PANEL_MIN_WIDTH          250
#define UI_PANEL_MIN_HEIGHT         120

#define UI_CARD_MIN_WIDTH           200
#define UI_CARD_MIN_HEIGHT          100

// =============================================================================
// DIMENSIONS - WIDGETS SPÉCIFIQUES
// =============================================================================
// Status badges
#define UI_BADGE_SIZE               16
#define UI_BADGE_BORDER             2

// Labels d'info (largeur du label "Nom:" dans les lignes)
#define UI_INFO_LABEL_WIDTH         120

// Hauteur des lignes d'info
#define UI_INFO_ROW_HEIGHT          LV_SIZE_CONTENT

// Largeur des colonnes (pourcentage)
#define UI_COL_WIDTH_PERCENT        48  // Pour layout 2 colonnes

// Progress bar (batterie, etc.)
#define UI_PROGRESS_HEIGHT          8
#define UI_PROGRESS_RADIUS          4

// Icons
#define UI_ICON_SIZE_SMALL          24
#define UI_ICON_SIZE_MEDIUM         32
#define UI_ICON_SIZE_LARGE          48

// =============================================================================
// OPACITÉ
// =============================================================================
#define UI_OPA_TRANSPARENT          LV_OPA_TRANSP
#define UI_OPA_LIGHT                LV_OPA_20
#define UI_OPA_MEDIUM               LV_OPA_50
#define UI_OPA_HEAVY                LV_OPA_80
#define UI_OPA_FULL                 LV_OPA_COVER

// Opacités spécifiques
#define UI_OPA_GLASS                LV_OPA_30   // Effet verre
#define UI_OPA_OVERLAY              LV_OPA_70   // Overlay/modal
#define UI_OPA_DISABLED             LV_OPA_40   // Éléments désactivés

// =============================================================================
// ANIMATIONS
// =============================================================================
#define UI_ANIM_TIME_INSTANT        0     // ms
#define UI_ANIM_TIME_FAST           150   // ms
#define UI_ANIM_TIME_NORMAL         300   // ms
#define UI_ANIM_TIME_SLOW           500   // ms
#define UI_ANIM_TIME_VERY_SLOW      800   // ms

// =============================================================================
// TIMERS
// =============================================================================
#define UI_TIMER_FAST               100   // ms (10 Hz)
#define UI_TIMER_NORMAL             500   // ms (2 Hz)
#define UI_TIMER_SLOW               1000  // ms (1 Hz)
#define UI_TIMER_VERY_SLOW          2000  // ms (0.5 Hz)

// =============================================================================
// EFFETS SPÉCIAUX
// =============================================================================
// Glow effect
#define UI_GLOW_COLOR               UI_COLOR_ACCENT_PRIMARY
#define UI_GLOW_WIDTH               3
#define UI_GLOW_OPA                 LV_OPA_50

// Outline effect
#define UI_OUTLINE_WIDTH            2
#define UI_OUTLINE_PAD              4
#define UI_OUTLINE_COLOR            UI_COLOR_ACCENT_PRIMARY

// =============================================================================
// LAYOUT PRESETS
// =============================================================================
// Header height
#define UI_HEADER_HEIGHT            60

// Footer height  
#define UI_FOOTER_HEIGHT            110

// Main content area (screen - header - footer)
#define UI_CONTENT_HEIGHT           410

#endif  // UI_CONFIG_H