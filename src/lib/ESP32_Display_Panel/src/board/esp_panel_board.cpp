/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <memory>
#include "utils/esp_panel_utils_log.h"
#include "esp_panel_board.hpp"
#include "esp_panel_board_private.hpp"
#include "esp_panel_board_default_config.hpp"

#undef _TO_DRIVERS_CLASS
#undef TO_DRIVERS_CLASS
#define _TO_DRIVERS_CLASS(type, name)  drivers::type ## name
#define TO_DRIVERS_CLASS(type, name)   _TO_DRIVERS_CLASS(type, name)

#undef _TO_STR
#undef TO_STR
#define _TO_STR(name) #name
#define TO_STR(name) _TO_STR(name)

namespace esp_panel::board {

    #if ESP_PANEL_BOARD_USE_DEFAULT
    Board::Board():
    Board(ESP_PANEL_BOARD_DEFAULT_CONFIG)
    {
        _use_default_config = true;
    }
    #else
    Board::Board()
    {
        _use_default_config = true;
    }
    #endif

    Board::~Board()
    {
        ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();
        ESP_UTILS_CHECK_FALSE_EXIT(del(), "Delete failed");
        ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();
    }

    bool Board::init()
    {
        ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();
        ESP_UTILS_CHECK_FALSE_RETURN(!isOverState(State::INIT), false, "Already initialized");

        if (!_config.isValid()) {
            #if !ESP_PANEL_BOARD_USE_DEFAULT
            ESP_UTILS_CHECK_FALSE_RETURN(
                !_use_default_config, false,
                "\nNo default board configuration detected. There are three ways to provide a default configuration: "
                "\n\t1. Use the `esp_panel_board_supported_conf.h` file to enable a supported board. "
                "\n\t2. Use the `esp_panel_board_custom_conf.h` file to define a custom board. "
                "\n\t3. Use menuconfig to enable a supported board or define a custom board. "
            );
            #endif
            ESP_UTILS_CHECK_FALSE_RETURN(false, false, "Invalid board configuration");
        }

        ESP_UTILS_LOGI("Initializing board (%s)", _config.name);

        // Create LCD device if it is used
        std::shared_ptr<drivers::Bus> lcd_bus = nullptr;
        std::shared_ptr<drivers::LCD> lcd_device = nullptr;
        if (isLCD_Used()) {
            auto &lcd_config = _config.lcd.value();
            ESP_UTILS_LOGD("Creating LCD (%s)", lcd_config.device_name);

            #if ESP_PANEL_BOARD_USE_DEFAULT && ESP_PANEL_BOARD_USE_LCD
            // If the LCD is configured by default, it will be created by the constructor
            if (_use_default_config) {
                using Bus_Class = TO_DRIVERS_CLASS(Bus, ESP_PANEL_BOARD_LCD_BUS_NAME);
                using LCD_Class = TO_DRIVERS_CLASS(LCD_, ESP_PANEL_BOARD_LCD_CONTROLLER);
                ESP_UTILS_CHECK_FALSE_RETURN(
                    std::holds_alternative<Bus_Class::Config>(lcd_config.bus_config), false,
                                             "LCD bus config is not a " TO_STR(ESP_PANEL_BOARD_LCD_BUS_NAME) " bus config"
                );
                ESP_UTILS_CHECK_EXCEPTION_RETURN(
                    (lcd_bus = utils::make_shared<Bus_Class>(std::get<Bus_Class::Config>(lcd_config.bus_config))), false,
                                                 "Create LCD bus failed"
                );
                ESP_UTILS_CHECK_EXCEPTION_RETURN(
                    (lcd_device = utils::make_shared<LCD_Class>(lcd_bus.get(), lcd_config.device_config)), false,
                                                 "Create LCD device failed"
                );
            }
            #endif // ESP_PANEL_BOARD_USE_DEFAULT && ESP_PANEL_BOARD_USE_LCD

            // If the LCD is not configured by default, it will be created by the factory function
            if (!_use_default_config) {
                lcd_device =
                drivers::LCD_Factory::create(lcd_config.device_name, lcd_config.bus_config, lcd_config.device_config);
                ESP_UTILS_CHECK_NULL_RETURN(lcd_device, false, "Create LCD device failed");
            }

            ESP_UTILS_CHECK_NULL_RETURN(lcd_device, false, "Create LCD failed");
            ESP_UTILS_LOGD("LCD create success");
        }

        // Create backlight device if it is used
        std::shared_ptr<drivers::Backlight> backlight = nullptr;
        if (isBacklightUsed()) {
            auto &backlight_config = _config.backlight.value();
            auto type = drivers::BacklightFactory::getConfigType(backlight_config.config);
            ESP_UTILS_LOGD("Creating backlight (%s[%d])", drivers::BacklightFactory::getTypeNameString(type).c_str(), type);

            #if ESP_PANEL_BOARD_USE_DEFAULT && ESP_PANEL_BOARD_USE_BACKLIGHT
            // If the backlight is configured by default, it will be created by the constructor
            if (_use_default_config) {
                using BacklightClass = TO_DRIVERS_CLASS(Backlight, ESP_PANEL_BOARD_BACKLIGHT_NAME);
                ESP_UTILS_CHECK_FALSE_RETURN(
                    std::holds_alternative<BacklightClass::Config>(backlight_config.config), false,
                                             "Backlight config is not a " TO_STR(ESP_PANEL_BOARD_BACKLIGHT_NAME) " backlight config"
                );
                ESP_UTILS_CHECK_EXCEPTION_RETURN(
                    (backlight =
                    utils::make_shared<BacklightClass>(std::get<BacklightClass::Config>(backlight_config.config))),
                                                 false, "Create backlight device failed"
                );
            }
            #endif // ESP_PANEL_BOARD_USE_DEFAULT && ESP_PANEL_BOARD_USE_BACKLIGHT

            // If the backlight is not configured by default, it will be created by the factory function
            if (!_use_default_config) {
                backlight = drivers::BacklightFactory::create(backlight_config.config);
                ESP_UTILS_CHECK_NULL_RETURN(backlight, false, "Create backlight device failed");
            }

            ESP_UTILS_CHECK_NULL_RETURN(backlight, false, "Create backlight failed");
            ESP_UTILS_LOGD("Backlight create success");
        }

        _lcd_bus = lcd_bus;
        _lcd_device = lcd_device;
        _backlight = backlight;

        setState(State::INIT);
        ESP_UTILS_LOGI("Board initialize success");
        ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();
        return true;
    }

    bool Board::begin()
    {
        ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();
        ESP_UTILS_CHECK_FALSE_RETURN(!isOverState(State::BEGIN), false, "Already begun");

        // Initialize the board if not initialized
        if (!isOverState(State::INIT)) {
            ESP_UTILS_CHECK_FALSE_RETURN(init(), false, "Init failed");
        }

        ESP_UTILS_LOGI("Beginning board (%s)", _config.name);
        auto &config = getConfig();

        if (config.stage_callbacks[BoardConfig::STAGE_CALLBACK_PRE_BOARD_BEGIN] != nullptr) {
            ESP_UTILS_LOGD("Board pre-begin");
            ESP_UTILS_CHECK_FALSE_RETURN(
                config.stage_callbacks[BoardConfig::STAGE_CALLBACK_PRE_BOARD_BEGIN](this), false, "Board pre-begin failed"
            );
        }

        // Begin the LCD if it is used
        auto lcd_device = getLCD();
        if (lcd_device != nullptr) {
            ESP_UTILS_LOGD("Beginning LCD");
            if (config.stage_callbacks[BoardConfig::STAGE_CALLBACK_PRE_LCD_BEGIN] != nullptr) {
                ESP_UTILS_LOGD("LCD pre-begin");
                ESP_UTILS_CHECK_FALSE_RETURN(
                    config.stage_callbacks[BoardConfig::STAGE_CALLBACK_PRE_LCD_BEGIN](this), false, "LCD pre-begin failed"
                );
            }

            ESP_UTILS_CHECK_FALSE_RETURN(lcd_device->begin(), false, "LCD device begin failed");

            if (lcd_device->isFunctionSupported(drivers::LCD::BasicBusSpecification::FUNC_DISPLAY_ON_OFF)) {
                ESP_UTILS_CHECK_FALSE_RETURN(lcd_device->setDisplayOnOff(true), false, "LCD device set display on failed");
            } else {
                ESP_UTILS_LOGD("LCD device doesn't support display on/off function");
            }

            auto &lcd_config = _config.lcd.value();
            if (lcd_device->isFunctionSupported(drivers::LCD::BasicBusSpecification::FUNC_INVERT_COLOR)) {
                ESP_UTILS_CHECK_FALSE_RETURN(
                    lcd_device->invertColor(lcd_config.pre_process.invert_color), false, "LCD device invert color failed"
                );
            } else {
                ESP_UTILS_LOGD("LCD device doesn't support invert color function");
            }

            if (lcd_device->isFunctionSupported(drivers::LCD::BasicBusSpecification::FUNC_SWAP_XY)) {
                ESP_UTILS_CHECK_FALSE_RETURN(
                    lcd_device->swapXY(lcd_config.pre_process.swap_xy), false, "LCD device swap XY failed"
                );
            } else {
                ESP_UTILS_LOGD("LCD device doesn't support swap XY function");
            }

            if (lcd_device->isFunctionSupported(drivers::LCD::BasicBusSpecification::FUNC_MIRROR_X)) {
                ESP_UTILS_CHECK_FALSE_RETURN(
                    lcd_device->mirrorX(lcd_config.pre_process.mirror_x), false, "LCD device mirror X failed"
                );
            } else {
                ESP_UTILS_LOGD("LCD device doesn't support mirror X function");
            }

            if (lcd_device->isFunctionSupported(drivers::LCD::BasicBusSpecification::FUNC_MIRROR_Y)) {
                ESP_UTILS_CHECK_FALSE_RETURN(
                    lcd_device->mirrorY(lcd_config.pre_process.mirror_y), false, "LCD device mirror Y failed"
                );
            } else {
                ESP_UTILS_LOGD("LCD device doesn't support mirror Y function");
            }

            if (lcd_device->isFunctionSupported(drivers::LCD::BasicBusSpecification::FUNC_GAP)) {
                ESP_UTILS_CHECK_FALSE_RETURN(
                    lcd_device->setGapX(lcd_config.pre_process.gap_x), false, "LCD device set gap X failed"
                );
                ESP_UTILS_CHECK_FALSE_RETURN(
                    lcd_device->setGapY(lcd_config.pre_process.gap_y), false, "LCD device set gap Y failed"
                );
            } else {
                ESP_UTILS_LOGD("LCD device doesn't support gap function");
            }

            if (config.stage_callbacks[BoardConfig::STAGE_CALLBACK_POST_LCD_BEGIN] != nullptr) {
                ESP_UTILS_LOGD("LCD post-begin");
                ESP_UTILS_CHECK_FALSE_RETURN(
                    config.stage_callbacks[BoardConfig::STAGE_CALLBACK_POST_LCD_BEGIN](this), false, "LCD post-begin failed"
                );
            }
            ESP_UTILS_LOGD("LCD begin success");
        }

        // Begin the backlight if it is used
        auto backlight = getBacklight();
        if (backlight != nullptr) {
            ESP_UTILS_LOGD("Beginning backlight");
            if (config.stage_callbacks[BoardConfig::STAGE_CALLBACK_PRE_BACKLIGHT_BEGIN] != nullptr) {
                ESP_UTILS_LOGD("Backlight pre-begin");
                ESP_UTILS_CHECK_FALSE_RETURN(
                    config.stage_callbacks[BoardConfig::STAGE_CALLBACK_PRE_BACKLIGHT_BEGIN](this), false,
                                             "Backlight pre-begin failed"
                );
            }

            auto &backlight_config = _config.backlight.value();
            ESP_UTILS_CHECK_FALSE_RETURN(backlight->begin(), false, "Backlight begin failed");

            if (backlight_config.pre_process.idle_off) {
                ESP_UTILS_CHECK_FALSE_RETURN(backlight->off(), false, "Backlight off failed");
            } else {
                ESP_UTILS_CHECK_FALSE_RETURN(backlight->on(), false, "Backlight on failed");
            }

            if (config.stage_callbacks[BoardConfig::STAGE_CALLBACK_POST_BACKLIGHT_BEGIN] != nullptr) {
                ESP_UTILS_LOGD("Backlight post-begin");
                ESP_UTILS_CHECK_FALSE_RETURN(
                    config.stage_callbacks[BoardConfig::STAGE_CALLBACK_POST_BACKLIGHT_BEGIN](this), false,
                                             "Backlight post-begin failed"
                );
            }
            ESP_UTILS_LOGD("Backlight begin success");
        }

        if (config.stage_callbacks[BoardConfig::STAGE_CALLBACK_POST_BOARD_BEGIN] != nullptr) {
            ESP_UTILS_LOGD("Board post-begin");
            ESP_UTILS_CHECK_FALSE_RETURN(
                config.stage_callbacks[BoardConfig::STAGE_CALLBACK_POST_BOARD_BEGIN](this), false, "Board post-begin failed"
            );
        }

        setState(State::BEGIN);
        ESP_UTILS_LOGI("Board begin success");
        ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();
        return true;
    }

    bool Board::del()
    {
        ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();
        auto &config = getConfig();

        if (!isOverState(State::INIT)) {
            goto end;
        }

        ESP_UTILS_LOGI("Deleting board (%s)", config.name);

        if (isOverState(State::BEGIN) && config.stage_callbacks[BoardConfig::STAGE_CALLBACK_PRE_BOARD_DEL] != nullptr) {
            ESP_UTILS_LOGD("Board pre-delete");
            ESP_UTILS_CHECK_FALSE_RETURN(
                config.stage_callbacks[BoardConfig::STAGE_CALLBACK_PRE_BOARD_DEL](this), false, "Board pre-delete failed"
            );
        }

        _backlight = nullptr;
        _lcd_device = nullptr;
        _lcd_bus = nullptr;

        if (isOverState(State::BEGIN) && config.stage_callbacks[BoardConfig::STAGE_CALLBACK_POST_BOARD_DEL] != nullptr) {
            ESP_UTILS_LOGD("Board post-delete");
            ESP_UTILS_CHECK_FALSE_RETURN(
                config.stage_callbacks[BoardConfig::STAGE_CALLBACK_POST_BOARD_DEL](this), false, "Board post-delete failed"
            );
        }

        setState(State::DEINIT);
        ESP_UTILS_LOGI("Board delete success");

        end:
        ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();
        return true;
    }

    bool Board::configCallback(board::BoardConfig::StageCallbackType type, BoardConfig::FunctionStageCallback callback)
    {
        ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();
        ESP_UTILS_CHECK_FALSE_RETURN(!isOverState(State::INIT), false, "Already initialized");
        ESP_UTILS_CHECK_FALSE_RETURN(type < BoardConfig::STAGE_CALLBACK_MAX, false, "Invalid callback type");

        _config.stage_callbacks[type] = callback;

        ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();
        return true;
    }

} // namespace esp_panel
