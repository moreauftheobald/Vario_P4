/**
 * @file Vario_P4.ino
 * @brief Point d'entrée du variomètre ESP32-P4
 */

#include <Arduino.h>
#include "src/hal/display_init.h"
#include "config/config.h"

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("===========================================");
    Serial.println("  VARIOMETER ESP32-P4");
    Serial.println("  v" PROJECT_VERSION);
    Serial.println("===========================================");
    
    // Init Board (Display + Touch)
    if (!display_init_board()) {
        Serial.println("[FATAL] Board init failed!");
        while (1) delay(1000);
    }
    
    // Init LVGL
    if (!display_init_lvgl()) {
        Serial.println("[FATAL] LVGL init failed!");
        while (1) delay(1000);
    }
    
    // Créer UI test
    lv_obj_t* scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
    
    lv_obj_t* label = lv_label_create(scr);
    lv_label_set_text(label, "VARIOMETER READY!");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0x00FF00), 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    
    lv_screen_load(scr);
    lv_obj_update_layout(scr);
    lv_refr_now(g_display);
    
    Serial.println("===========================================");
    Serial.println("  READY");
    Serial.println("===========================================");
}

void loop() {
    display_task();
    delay(1);
}