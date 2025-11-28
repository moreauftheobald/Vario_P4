/**
 * @file ui_helper.h
 * @brief Fonctions helper pour création d'éléments UI (Design moderne)
 */

#ifndef UI_HELPER_H
#define UI_HELPER_H

#include <lvgl.h>
#include "src/config/ui_config.h"

// =============================================================================
// ÉCRANS ET CONTAINERS
// =============================================================================

/**
 * @brief Crée un écran avec fond dégradé
 */
static inline lv_obj_t* ui_create_screen() {
  lv_obj_t* screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(screen, lv_color_hex(UI_COLOR_BG), 0);
  lv_obj_set_style_bg_grad_color(screen, lv_color_hex(UI_COLOR_BG_SECONDARY), 0);
  lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER, 0);
  return screen;
}

/**
 * @brief Crée un container transparent
 */
static inline lv_obj_t* ui_create_container(lv_obj_t* parent) {
  lv_obj_t* cont = lv_obj_create(parent);
  lv_obj_set_style_bg_opa(cont, UI_OPA_TRANSPARENT, 0);
  lv_obj_set_style_border_width(cont, UI_BORDER_NONE, 0);
  lv_obj_set_style_pad_all(cont, UI_PAD_NONE, 0);
  lv_obj_set_style_shadow_width(cont, 0, 0);
  return cont;
}

/**
 * @brief Crée un container avec flex layout
 */
static inline lv_obj_t* ui_create_flex_container(lv_obj_t* parent, 
                                                  lv_flex_flow_t flow,
                                                  lv_flex_align_t main_align,
                                                  lv_flex_align_t cross_align) {
  lv_obj_t* cont = ui_create_container(parent);
  lv_obj_set_flex_flow(cont, flow);
  lv_obj_set_flex_align(cont, main_align, cross_align, LV_FLEX_ALIGN_START);
  return cont;
}

// =============================================================================
// PANNEAUX ET CARTES MODERNES
// =============================================================================

/**
 * @brief Crée une carte moderne avec ombre
 */
static inline lv_obj_t* ui_create_card(lv_obj_t* parent,
                                       uint32_t bg_color,
                                       uint8_t shadow_width) {
  lv_obj_t* card = lv_obj_create(parent);
  
  // Style de base
  lv_obj_set_style_bg_color(card, lv_color_hex(bg_color), 0);
  lv_obj_set_style_border_width(card, UI_BORDER_THIN, 0);
  lv_obj_set_style_border_color(card, lv_color_hex(UI_COLOR_PANEL_BORDER), 0);
  lv_obj_set_style_border_opa(card, UI_OPA_LIGHT, 0);
  lv_obj_set_style_radius(card, UI_RADIUS_LARGE, 0);
  lv_obj_set_style_pad_all(card, UI_PAD_MEDIUM, 0);
  
  // Ombre
  lv_obj_set_style_shadow_width(card, shadow_width, 0);
  lv_obj_set_style_shadow_color(card, lv_color_hex(UI_SHADOW_COLOR), 0);
  lv_obj_set_style_shadow_opa(card, UI_SHADOW_OPA_MEDIUM, 0);
  lv_obj_set_style_shadow_ofs_x(card, UI_SHADOW_OFS_X, 0);
  lv_obj_set_style_shadow_ofs_y(card, UI_SHADOW_OFS_Y, 0);
  lv_obj_set_style_shadow_spread(card, UI_SHADOW_SPREAD, 0);
  
  return card;
}

/**
 * @brief Crée une carte avec titre et icône
 */
static inline lv_obj_t* ui_create_titled_card(lv_obj_t* parent,
                                              const char* title,
                                              const char* icon,
                                              uint32_t title_color) {
  lv_obj_t* card = ui_create_card(parent, UI_COLOR_PANEL, UI_SHADOW_WIDTH_MEDIUM);
  lv_obj_set_size(card, lv_pct(100), LV_SIZE_CONTENT);
  lv_obj_set_flex_flow(card, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(card, 0, 0);  // Supprimer l'espace entre les enfants
  
  // Container titre avec icône
  lv_obj_t* title_cont = ui_create_flex_container(card, LV_FLEX_FLOW_ROW,
                                                  LV_FLEX_ALIGN_START,
                                                  LV_FLEX_ALIGN_CENTER);
  lv_obj_set_width(title_cont, lv_pct(100));
  lv_obj_set_height(title_cont, LV_SIZE_CONTENT);
  
  // Icône
  if (icon) {
    lv_obj_t* label_icon = lv_label_create(title_cont);
    lv_label_set_text(label_icon, icon);
    lv_obj_set_style_text_font(label_icon, UI_FONT_XLARGE, 0);
    lv_obj_set_style_text_color(label_icon, lv_color_hex(title_color), 0);
  }
  
  // Titre
  lv_obj_t* label_title = lv_label_create(title_cont);
  lv_label_set_text(label_title, title);
  lv_obj_set_style_text_font(label_title, UI_FONT_LARGE, 0);
  lv_obj_set_style_text_color(label_title, lv_color_hex(title_color), 0);
  lv_obj_set_style_pad_left(label_title, UI_PAD_SMALL, 0);
  
  // Séparateur moderne (marges minimales)
  lv_obj_t* sep = lv_obj_create(card);
  lv_obj_set_size(sep, lv_pct(100), 2);
  lv_obj_set_style_bg_color(sep, lv_color_hex(title_color), 0);
  lv_obj_set_style_bg_grad_color(sep, lv_color_hex(UI_COLOR_BG), 0);
  lv_obj_set_style_bg_grad_dir(sep, LV_GRAD_DIR_HOR, 0);
  lv_obj_set_style_border_width(sep, 0, 0);
  lv_obj_set_style_pad_all(sep, 0, 0);
  lv_obj_set_style_radius(sep, UI_RADIUS_SMALL, 0);
  lv_obj_set_style_margin_top(sep, 4, 0);      // Marge top minimale
  lv_obj_set_style_margin_bottom(sep, 8, 0);   // Marge bottom
  
  return card;
}

// =============================================================================
// LABELS ET TEXTES
// =============================================================================

/**
 * @brief Crée un label avec style
 */
static inline lv_obj_t* ui_create_label(lv_obj_t* parent,
                                        const char* text,
                                        const lv_font_t* font,
                                        uint32_t color) {
  lv_obj_t* label = lv_label_create(parent);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
  return label;
}

/**
 * @brief Crée un label avec glow effect
 */
static inline lv_obj_t* ui_create_glow_label(lv_obj_t* parent,
                                             const char* text,
                                             const lv_font_t* font,
                                             uint32_t color) {
  lv_obj_t* label = ui_create_label(parent, text, font, color);
  
  // Effet glow avec outline
  lv_obj_set_style_outline_width(label, UI_GLOW_WIDTH, 0);
  lv_obj_set_style_outline_color(label, lv_color_hex(color), 0);
  lv_obj_set_style_outline_opa(label, UI_GLOW_OPA, 0);
  lv_obj_set_style_outline_pad(label, 0, 0);
  
  return label;
}

// =============================================================================
// BADGES DE STATUT
// =============================================================================

/**
 * @brief Crée un badge de statut circulaire
 */
static inline lv_obj_t* ui_create_status_badge(lv_obj_t* parent,
                                               uint32_t color,
                                               bool pulsing) {
  lv_obj_t* badge = lv_obj_create(parent);
  lv_obj_set_size(badge, UI_BADGE_SIZE, UI_BADGE_SIZE);
  lv_obj_set_style_radius(badge, UI_RADIUS_ROUND, 0);
  lv_obj_set_style_bg_color(badge, lv_color_hex(color), 0);
  lv_obj_set_style_border_width(badge, UI_BADGE_BORDER, 0);
  lv_obj_set_style_border_color(badge, lv_color_hex(UI_COLOR_BG), 0);
  lv_obj_set_style_pad_all(badge, 0, 0);
  
  // Effet glow
  lv_obj_set_style_shadow_width(badge, 8, 0);
  lv_obj_set_style_shadow_color(badge, lv_color_hex(color), 0);
  lv_obj_set_style_shadow_opa(badge, UI_OPA_HEAVY, 0);
  lv_obj_set_style_shadow_spread(badge, 2, 0);
  
  // TODO: Animation pulsing si nécessaire
  
  return badge;
}

/**
 * @brief Crée une ligne de statut avec badge
 */
static inline lv_obj_t* ui_create_status_row(lv_obj_t* parent,
                                             const char* icon,
                                             const char* label_text,
                                             const char* value_text,
                                             uint32_t badge_color) {
  lv_obj_t* row = ui_create_flex_container(parent, LV_FLEX_FLOW_ROW,
                                           LV_FLEX_ALIGN_START,
                                           LV_FLEX_ALIGN_CENTER);
  lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
  lv_obj_set_style_pad_all(row, UI_PAD_TINY, 0);
  
  // Badge de statut
  lv_obj_t* badge = ui_create_status_badge(row, badge_color, false);
  
  // Icône
  if (icon) {
    lv_obj_t* icon_label = ui_create_label(row, icon, UI_FONT_MEDIUM, UI_COLOR_TEXT_SECONDARY);
    lv_obj_set_style_pad_left(icon_label, UI_PAD_SMALL, 0);
  }
  
  // Label
  lv_obj_t* label = ui_create_label(row, label_text, UI_FONT_NORMAL, UI_COLOR_TEXT_PRIMARY);
  lv_obj_set_style_pad_left(label, UI_PAD_SMALL, 0);
  
  // Spacer
  lv_obj_t* spacer = lv_obj_create(row);
  lv_obj_set_size(spacer, lv_pct(100), 1);
  lv_obj_set_style_bg_opa(spacer, UI_OPA_TRANSPARENT, 0);
  lv_obj_set_flex_grow(spacer, 1);
  
  // Valeur
  lv_obj_t* value = ui_create_label(row, value_text, UI_FONT_NORMAL, UI_COLOR_ACCENT_PRIMARY);
  
  return row;
}

// =============================================================================
// LIGNES D'INFO AMÉLIORÉES
// =============================================================================

/**
 * @brief Crée une ligne d'information moderne
 */
static inline lv_obj_t* ui_create_info_row(lv_obj_t* parent,
                                           const char* label_text,
                                           const char* value_text) {
  lv_obj_t* row = ui_create_flex_container(parent, LV_FLEX_FLOW_ROW,
                                           LV_FLEX_ALIGN_SPACE_BETWEEN,
                                           LV_FLEX_ALIGN_CENTER);
  lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
  lv_obj_set_style_pad_all(row, UI_PAD_TINY, 0);
  
  // Label
  ui_create_label(row, label_text, UI_FONT_MEDIUM, UI_COLOR_TEXT_SECONDARY);
  
  // Valeur (alignée à droite)
  ui_create_label(row, value_text, UI_FONT_MEDIUM, UI_COLOR_TEXT_PRIMARY);
  
  return row;
}

// =============================================================================
// PROGRESS BARS ET INDICATEURS
// =============================================================================

/**
 * @brief Crée une barre de progression moderne
 */
static inline lv_obj_t* ui_create_progress_bar(lv_obj_t* parent,
                                               int32_t value,
                                               uint32_t color) {
  lv_obj_t* bar = lv_bar_create(parent);
  lv_obj_set_size(bar, lv_pct(100), UI_PROGRESS_HEIGHT);
  lv_obj_set_style_radius(bar, UI_PROGRESS_RADIUS, 0);
  
  // Background
  lv_obj_set_style_bg_color(bar, lv_color_hex(UI_COLOR_BG_SECONDARY), 0);
  lv_obj_set_style_bg_opa(bar, UI_OPA_FULL, 0);
  
  // Indicateur avec gradient
  lv_obj_set_style_bg_color(bar, lv_color_hex(color), LV_PART_INDICATOR);
  lv_obj_set_style_bg_grad_color(bar, lv_color_lighten(lv_color_hex(color), 50), LV_PART_INDICATOR);
  lv_obj_set_style_bg_grad_dir(bar, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
  lv_obj_set_style_radius(bar, UI_PROGRESS_RADIUS, LV_PART_INDICATOR);
  
  // Glow effect
  lv_obj_set_style_shadow_width(bar, 6, LV_PART_INDICATOR);
  lv_obj_set_style_shadow_color(bar, lv_color_hex(color), LV_PART_INDICATOR);
  lv_obj_set_style_shadow_opa(bar, UI_OPA_MEDIUM, LV_PART_INDICATOR);
  
  lv_bar_set_value(bar, value, LV_ANIM_OFF);
  
  return bar;
}

/**
 * @brief Crée une ligne avec label, valeur et barre de progression
 */
static inline lv_obj_t* ui_create_progress_row(lv_obj_t* parent,
                                               const char* label_text,
                                               int32_t value,
                                               uint32_t color) {
  lv_obj_t* cont = ui_create_flex_container(parent, LV_FLEX_FLOW_COLUMN,
                                            LV_FLEX_ALIGN_START,
                                            LV_FLEX_ALIGN_START);
  lv_obj_set_size(cont, lv_pct(100), LV_SIZE_CONTENT);
  lv_obj_set_style_pad_row(cont, UI_PAD_TINY, 0);
  
  // Header avec label et valeur
  lv_obj_t* header = ui_create_flex_container(cont, LV_FLEX_FLOW_ROW,
                                              LV_FLEX_ALIGN_SPACE_BETWEEN,
                                              LV_FLEX_ALIGN_CENTER);
  lv_obj_set_width(header, lv_pct(100));
  
  ui_create_label(header, label_text, UI_FONT_SMALL, UI_COLOR_TEXT_SECONDARY);
  
  char value_str[16];
  snprintf(value_str, sizeof(value_str), "%d%%", (int)value);
  ui_create_label(header, value_str, UI_FONT_SMALL, color);
  
  // Barre de progression
  ui_create_progress_bar(cont, value, color);
  
  return cont;
}

// =============================================================================
// BOUTONS MODERNES
// =============================================================================

/**
 * @brief Crée un bouton moderne avec gradient
 */
static inline lv_obj_t* ui_create_button(lv_obj_t* parent,
                                         const char* text,
                                         const char* symbol,
                                         uint32_t bg_color,
                                         uint16_t width,
                                         uint16_t height,
                                         const lv_font_t* font,
                                         lv_event_cb_t event_cb,
                                         void* user_data) {
  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_set_size(btn, width, height);
  
  // Style normal avec gradient
  lv_obj_set_style_bg_color(btn, lv_color_hex(bg_color), 0);
  lv_obj_set_style_bg_grad_color(btn, lv_color_lighten(lv_color_hex(bg_color), 30), 0);
  lv_obj_set_style_bg_grad_dir(btn, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_radius(btn, UI_RADIUS_MEDIUM, 0);
  lv_obj_set_style_border_width(btn, UI_BORDER_THIN, 0);
  lv_obj_set_style_border_color(btn, lv_color_lighten(lv_color_hex(bg_color), 50), 0);
  lv_obj_set_style_border_opa(btn, UI_OPA_MEDIUM, 0);
  
  // Ombre
  lv_obj_set_style_shadow_width(btn, UI_SHADOW_WIDTH_SMALL, 0);
  lv_obj_set_style_shadow_color(btn, lv_color_hex(UI_SHADOW_COLOR), 0);
  lv_obj_set_style_shadow_opa(btn, UI_SHADOW_OPA_MEDIUM, 0);
  lv_obj_set_style_shadow_ofs_y(btn, UI_SHADOW_OFS_Y, 0);
  
  // Style pressed
  lv_obj_set_style_bg_color(btn, lv_color_darken(lv_color_hex(bg_color), 30), LV_STATE_PRESSED);
  lv_obj_set_style_shadow_ofs_y(btn, 2, LV_STATE_PRESSED);
  
  // Style disabled
  lv_obj_set_style_bg_opa(btn, UI_OPA_DISABLED, LV_STATE_DISABLED);
  
  if (event_cb) {
    lv_obj_add_event_cb(btn, event_cb, LV_EVENT_CLICKED, user_data);
  }
  
  // Container flex pour centrer verticalement et horizontalement
  lv_obj_t* cont = lv_obj_create(btn);
  lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
  lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(cont, 0, 0);
  lv_obj_set_style_pad_all(cont, 0, 0);
  lv_obj_set_style_shadow_width(cont, 0, 0);
  lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  
  // Icône
  if (symbol) {
    lv_obj_t* icon = lv_label_create(cont);
    lv_label_set_text(icon, symbol);
    lv_obj_set_style_text_font(icon, font, 0);
    lv_obj_set_style_text_color(icon, lv_color_hex(UI_COLOR_TEXT_PRIMARY), 0);
  }
  
  // Texte
  lv_obj_t* label = lv_label_create(cont);
  lv_label_set_text(label, text);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, lv_color_hex(UI_COLOR_TEXT_PRIMARY), 0);
  if (symbol) {
    lv_obj_set_style_pad_left(label, UI_PAD_SMALL, 0);
  }
  
  return btn;
}

// =============================================================================
// HEADER / TITRE
// =============================================================================

/**
 * @brief Crée un header moderne sans cadre
 */
static inline lv_obj_t* ui_create_header(lv_obj_t* parent, const char* title) {
  lv_obj_t* header = lv_obj_create(parent);
  lv_obj_set_size(header, lv_pct(100), UI_HEADER_HEIGHT);
  
  // Supprimer TOUS les styles et paddings
  lv_obj_set_style_bg_opa(header, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(header, 0, 0);
  lv_obj_set_style_shadow_width(header, 0, 0);
  lv_obj_set_style_outline_width(header, 0, 0);
  lv_obj_set_style_radius(header, 0, 0);
  lv_obj_set_style_pad_all(header, 0, 0);  // Padding à zéro
  
  // Titre avec effet glow centré
  lv_obj_t* label = ui_create_glow_label(header, title, UI_FONT_HUGE, UI_COLOR_ACCENT_PRIMARY);
  lv_obj_center(label);
  
  return header;
}

// =============================================================================
// LAYOUT HELPERS
// =============================================================================

/**
 * @brief Configure un layout 2 colonnes
 */
static inline void ui_setup_two_column_layout(lv_obj_t* parent,
                                              lv_obj_t** left_col,
                                              lv_obj_t** right_col) {
  lv_obj_set_flex_flow(parent, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(parent, LV_FLEX_ALIGN_SPACE_BETWEEN, 
                       LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_set_style_pad_column(parent, UI_GAP_LARGE, 0);
  
  *left_col = ui_create_container(parent);
  lv_obj_set_size(*left_col, lv_pct(UI_COL_WIDTH_PERCENT), lv_pct(100));
  lv_obj_set_flex_flow(*left_col, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(*left_col, UI_GAP_MEDIUM, 0);
  
  *right_col = ui_create_container(parent);
  lv_obj_set_size(*right_col, lv_pct(UI_COL_WIDTH_PERCENT), lv_pct(100));
  lv_obj_set_flex_flow(*right_col, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(*right_col, UI_GAP_MEDIUM, 0);
}

/**
 * @brief Crée un container de boutons (horizontal, espacés)
 */
static inline lv_obj_t* ui_create_button_container(lv_obj_t* parent) {
  lv_obj_t* cont = ui_create_container(parent);
  lv_obj_set_size(cont, lv_pct(100), UI_FOOTER_HEIGHT);
  lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_SPACE_EVENLY, 
                       LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(cont, UI_PAD_MEDIUM, 0);
  return cont;
}

// =============================================================================
// UTILITAIRES
// =============================================================================

/**
 * @brief Active/désactive un objet
 */
static inline void ui_set_enabled(lv_obj_t* obj, bool enabled) {
  if (enabled) {
    lv_obj_clear_state(obj, LV_STATE_DISABLED);
  } else {
    lv_obj_add_state(obj, LV_STATE_DISABLED);
  }
}

/**
 * @brief Affiche/cache un objet
 */
static inline void ui_set_visible(lv_obj_t* obj, bool visible) {
  if (visible) {
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
  }
}

/**
 * @brief Met à jour la valeur d'une barre de progression avec animation
 */
static inline void ui_update_progress_bar(lv_obj_t* bar, int32_t value) {
  lv_bar_set_value(bar, value, LV_ANIM_ON);
}

#endif  // UI_HELPER_H