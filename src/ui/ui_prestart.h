/**
 * @file ui_prestart.h
 * @brief Écran de préstart - VERSION SIMPLE QUI MARCHE
 */

#ifndef UI_PRESTART_H
#define UI_PRESTART_H

#include <lvgl.h>
#include "src/config/config.h"
#include "src/config/ui_config.h"
#include "src/system/logger.h"
#include "src/system/lang.h"
#include "src/system/flight_data.h"
#include "src/hal/sd_helper.h"
#include "src/ui/ui_helper.h"

// =============================================================================
// VARIABLES GLOBALES
// =============================================================================
static lv_obj_t* prestart_screen = NULL;
static lv_obj_t* status_rows[7] = {NULL};
static lv_obj_t* battery_progress = NULL;
static lv_obj_t* btn_start = NULL;
static lv_timer_t* status_update_timer = NULL;

// =============================================================================
// CALLBACKS BOUTONS
// =============================================================================

static void btn_start_cb(lv_event_t* e) {
  LOG_I(LOG_UI, "Start button pressed");
  
  if (status_update_timer) {
    lv_timer_del(status_update_timer);
    status_update_timer = NULL;
  }
}

static void btn_settings_cb(lv_event_t* e) {
  LOG_I(LOG_UI, "Settings button pressed");
  
  if (status_update_timer) {
    lv_timer_del(status_update_timer);
    status_update_timer = NULL;
  }
}

static void btn_file_transfer_cb(lv_event_t* e) {
  LOG_I(LOG_UI, "File transfer button pressed");
  
  if (status_update_timer) {
    lv_timer_del(status_update_timer);
    status_update_timer = NULL;
  }
}

// =============================================================================
// MISE À JOUR STATUTS
// =============================================================================

static void update_status_row(lv_obj_t* row, const char* value_text, uint32_t badge_color) {
  if (!row || !lv_obj_is_valid(row)) return;
  
  lv_obj_t* badge = lv_obj_get_child(row, 0);
  if (badge) {
    lv_obj_set_style_bg_color(badge, lv_color_hex(badge_color), 0);
    lv_obj_set_style_shadow_color(badge, lv_color_hex(badge_color), 0);
  }
  
  uint32_t child_count = lv_obj_get_child_cnt(row);
  if (child_count > 0) {
    lv_obj_t* value_label = lv_obj_get_child(row, child_count - 1);
    lv_label_set_text(value_label, value_text);
  }
}

static void update_sensor_status() {
  char buf[128];
  const char** txt = trad[current_lang];
  
  if (sd_initialized) {
    uint64_t total = sd_get_total_space() / (1024 * 1024 * 1024);
    uint64_t free = sd_get_free_space() / (1024 * 1024 * 1024);
    snprintf(buf, sizeof(buf), "%lluGB / %lluGB", free, total);
    update_status_row(status_rows[0], buf, UI_COLOR_SUCCESS);
  } else {
    update_status_row(status_rows[0], txt[30], UI_COLOR_ERROR);
  }
  
  bool baro_ok = true;
  update_status_row(status_rows[1], baro_ok ? txt[29] : txt[30], 
                   baro_ok ? UI_COLOR_SUCCESS : UI_COLOR_ERROR);
  
  bool imu_ok = true;
  update_status_row(status_rows[2], imu_ok ? txt[29] : txt[30],
                   imu_ok ? UI_COLOR_SUCCESS : UI_COLOR_ERROR);
  
  FlightData fd;
  bool has_data = flight_data_copy(&fd);
  
  if (has_data && fd.gps_fix) {
    snprintf(buf, sizeof(buf), "%s - %d %s", txt[32], fd.satellites, txt[36]);
    update_status_row(status_rows[3], buf, UI_COLOR_SUCCESS);
  } else if (has_data) {
    snprintf(buf, sizeof(buf), "%s - %d %s", txt[31], fd.satellites, txt[36]);
    update_status_row(status_rows[3], buf, UI_COLOR_WARNING);
  } else {
    update_status_row(status_rows[3], txt[33], UI_COLOR_ERROR);
  }
  
  bool wifi_connected = wifi_is_connected();
  if (wifi_connected) {
    String ip = wifi_get_local_ip();
    update_status_row(status_rows[4], ip.c_str(), UI_COLOR_SUCCESS);
  } else {
    update_status_row(status_rows[4], txt[34], UI_COLOR_WARNING);
  }
  
  if (has_data && battery_progress) {
    snprintf(buf, sizeof(buf), "%.1f%%", fd.battery_percent);
    update_status_row(status_rows[5], buf, 
                     fd.battery_percent > 50 ? UI_COLOR_SUCCESS : 
                     fd.battery_percent > 20 ? UI_COLOR_WARNING : UI_COLOR_ERROR);
    
    uint32_t bar_color = fd.battery_percent > 50 ? UI_COLOR_SUCCESS : 
                        fd.battery_percent > 20 ? UI_COLOR_WARNING : UI_COLOR_ERROR;
    lv_obj_set_style_bg_color(battery_progress, lv_color_hex(bar_color), LV_PART_INDICATOR);
    lv_obj_set_style_bg_grad_color(battery_progress, 
                                   lv_color_lighten(lv_color_hex(bar_color), 50), 
                                   LV_PART_INDICATOR);
    ui_update_progress_bar(battery_progress, (int32_t)fd.battery_percent);
  }
  
  bool kalman_ok = true;
  update_status_row(status_rows[6], kalman_ok ? txt[29] : txt[33],
                   kalman_ok ? UI_COLOR_SUCCESS : UI_COLOR_WARNING);
  
  bool can_start = sd_initialized && baro_ok && imu_ok && kalman_ok;
  ui_set_enabled(btn_start, can_start);
}

static void status_update_timer_cb(lv_timer_t* timer) {
  if (!prestart_screen || !lv_obj_is_valid(prestart_screen)) {
    if (timer) {
      lv_timer_del(timer);
      status_update_timer = NULL;
    }
    return;
  }
  
  update_sensor_status();
}

// =============================================================================
// CRÉATION UI
// =============================================================================

void prestart_create() {
  LOG_I(LOG_UI, "Creating prestart screen");
  
  const char** txt = trad[current_lang];
  
  prestart_screen = ui_create_screen();
  
  // === HEADER ===
  lv_obj_t* header = ui_create_header(prestart_screen, txt[0]);
  lv_obj_align(header, LV_ALIGN_TOP_MID, 0, 0);
  
  // === CONTAINER PRINCIPAL ===
  lv_obj_t* main_cont = ui_create_container(prestart_screen);
  lv_obj_set_size(main_cont, lv_pct(96), UI_CONTENT_HEIGHT);
  lv_obj_align(main_cont, LV_ALIGN_TOP_MID, 0, UI_HEADER_HEIGHT + UI_PAD_SMALL);
  
  lv_obj_t* left_col, *right_col;
  ui_setup_two_column_layout(main_cont, &left_col, &right_col);
  
  // Forcer la hauteur fixe des colonnes
  lv_obj_set_height(left_col, UI_CONTENT_HEIGHT);
  lv_obj_set_height(right_col, UI_CONTENT_HEIGHT);
  
  // === COLONNE GAUCHE: INFO SYSTÈME ===
  lv_obj_t* system_card = ui_create_titled_card(left_col, txt[11], 
                                                 LV_SYMBOL_SETTINGS,
                                                 UI_COLOR_ACCENT_PRIMARY);
  lv_obj_set_height(system_card, lv_pct(100));
  
  // Changer l'alignement flex pour espacer uniformément
  lv_obj_set_flex_align(system_card, LV_FLEX_ALIGN_SPACE_EVENLY, 
                       LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  
  // Version
  char version_buf[64];
  snprintf(version_buf, sizeof(version_buf), "%s %s", txt[12], PROJECT_VERSION);
  lv_obj_t* version_label = ui_create_label(system_card, version_buf, 
                                            UI_FONT_SMALL, UI_COLOR_TEXT_SECONDARY);
  
  // Status rows
  status_rows[0] = ui_create_status_row(system_card, LV_SYMBOL_SD_CARD, 
                                        txt[13], "---", UI_COLOR_BADGE_NEUTRAL);
  
  status_rows[1] = ui_create_status_row(system_card, LV_SYMBOL_SETTINGS,
                                        txt[14], "---", UI_COLOR_BADGE_NEUTRAL);
  
  status_rows[2] = ui_create_status_row(system_card, LV_SYMBOL_GPS,
                                        txt[15], "---", UI_COLOR_BADGE_NEUTRAL);
  
  status_rows[3] = ui_create_status_row(system_card, LV_SYMBOL_GPS,
                                        txt[16], "---", UI_COLOR_BADGE_NEUTRAL);
  
  status_rows[4] = ui_create_status_row(system_card, LV_SYMBOL_WIFI,
                                        txt[17], "---", UI_COLOR_BADGE_NEUTRAL);
  
  status_rows[5] = ui_create_status_row(system_card, LV_SYMBOL_BATTERY_FULL,
                                        txt[18], "---", UI_COLOR_BADGE_NEUTRAL);
  
  battery_progress = ui_create_progress_bar(system_card, 0, UI_COLOR_SUCCESS);
  
  status_rows[6] = ui_create_status_row(system_card, LV_SYMBOL_SETTINGS,
                                        txt[19], "---", UI_COLOR_BADGE_NEUTRAL);
  
  // === COLONNE DROITE: PILOTE ET ICE ===
  lv_obj_t* pilot_card = ui_create_titled_card(right_col, txt[20],
                                                LV_SYMBOL_HOME,
                                                UI_COLOR_ACCENT_PRIMARY);
  lv_obj_set_height(pilot_card, lv_pct(100));
  
  // Changer l'alignement flex pour espacer uniformément
  lv_obj_set_flex_align(pilot_card, LV_FLEX_ALIGN_SPACE_EVENLY, 
                       LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  
  ui_create_info_row(pilot_card, txt[21], "---");
  ui_create_info_row(pilot_card, txt[22], "---");
  ui_create_info_row(pilot_card, txt[23], "---");
  ui_create_info_row(pilot_card, txt[24], "---");
  
  // Section ICE - créer un mini-groupe titre+séparateur+lignes
  lv_obj_t* ice_group = lv_obj_create(pilot_card);
  lv_obj_set_size(ice_group, lv_pct(100), LV_SIZE_CONTENT);
  lv_obj_set_style_bg_opa(ice_group, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(ice_group, 0, 0);
  lv_obj_set_style_pad_all(ice_group, 0, 0);
  lv_obj_set_style_shadow_width(ice_group, 0, 0);
  lv_obj_set_flex_flow(ice_group, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_row(ice_group, 0, 0);
  
  lv_obj_t* ice_title_cont = ui_create_flex_container(ice_group, LV_FLEX_FLOW_ROW,
                                                      LV_FLEX_ALIGN_START,
                                                      LV_FLEX_ALIGN_CENTER);
  lv_obj_set_width(ice_title_cont, lv_pct(100));
  
  lv_obj_t* ice_icon = lv_label_create(ice_title_cont);
  lv_label_set_text(ice_icon, LV_SYMBOL_WARNING);
  lv_obj_set_style_text_font(ice_icon, UI_FONT_XLARGE, 0);
  lv_obj_set_style_text_color(ice_icon, lv_color_hex(UI_COLOR_ERROR), 0);
  
  lv_obj_t* ice_title = lv_label_create(ice_title_cont);
  lv_label_set_text(ice_title, txt[25]);
  lv_obj_set_style_text_font(ice_title, UI_FONT_LARGE, 0);
  lv_obj_set_style_text_color(ice_title, lv_color_hex(UI_COLOR_ERROR), 0);
  lv_obj_set_style_pad_left(ice_title, UI_PAD_SMALL, 0);
  
  lv_obj_t* ice_sep = lv_obj_create(ice_group);
  lv_obj_set_size(ice_sep, lv_pct(100), 2);
  lv_obj_set_style_bg_color(ice_sep, lv_color_hex(UI_COLOR_ERROR), 0);
  lv_obj_set_style_bg_grad_color(ice_sep, lv_color_hex(UI_COLOR_BG), 0);
  lv_obj_set_style_bg_grad_dir(ice_sep, LV_GRAD_DIR_HOR, 0);
  lv_obj_set_style_border_width(ice_sep, 0, 0);
  lv_obj_set_style_pad_all(ice_sep, 0, 0);
  lv_obj_set_style_radius(ice_sep, UI_RADIUS_SMALL, 0);
  lv_obj_set_style_margin_top(ice_sep, 4, 0);
  lv_obj_set_style_margin_bottom(ice_sep, 8, 0);
  
  ui_create_info_row(ice_group, txt[21], "---");
  ui_create_info_row(ice_group, txt[22], "---");
  ui_create_info_row(ice_group, txt[23], "---");
  
  // === FOOTER AVEC BOUTONS ===
  lv_obj_t* footer = ui_create_button_container(prestart_screen);
  lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, 0);
  
  btn_start = ui_create_button(footer, txt[26], LV_SYMBOL_PLAY,
                               UI_COLOR_BTN_START,
                               UI_BTN_PRESTART_WIDTH, UI_BTN_PRESTART_HEIGHT,
                               UI_FONT_LARGE, btn_start_cb, NULL);
  
  ui_create_button(footer, txt[27], LV_SYMBOL_SETTINGS,
                  UI_COLOR_BTN_SETTINGS,
                  UI_BTN_PRESTART_WIDTH, UI_BTN_PRESTART_HEIGHT,
                  UI_FONT_LARGE, btn_settings_cb, NULL);
  
  ui_create_button(footer, txt[28], LV_SYMBOL_USB,
                  UI_COLOR_BTN_FILES,
                  UI_BTN_PRESTART_WIDTH, UI_BTN_PRESTART_HEIGHT,
                  UI_FONT_LARGE, btn_file_transfer_cb, NULL);
  
  lv_screen_load(prestart_screen);
  update_sensor_status();
  
  status_update_timer = lv_timer_create(status_update_timer_cb, UI_TIMER_SLOW, NULL);
  
  LOG_I(LOG_UI, "Prestart screen created");
}

void prestart_show() {
  prestart_create();
}

void prestart_close() {
  if (status_update_timer) {
    lv_timer_del(status_update_timer);
    status_update_timer = NULL;
  }
  
  if (prestart_screen) {
    lv_obj_del(prestart_screen);
    prestart_screen = NULL;
  }
  
  LOG_I(LOG_UI, "Prestart screen closed");
}

#endif  // UI_PRESTART_H