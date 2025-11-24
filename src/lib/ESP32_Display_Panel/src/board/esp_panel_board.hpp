/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <memory>
#include <string>
#include "esp_panel_types.h"
#include "utils/esp_panel_utils_cxx.hpp"
#include "esp_panel_board_conf_internal.h"
#include "esp_panel_board_config.hpp"

namespace esp_panel::board {

    /**
     * @brief Panel device class for ESP development boards
     *
     * This class integrates independent drivers such as LCD and Backlight for development boards.
     */
    class Board {
    public:
        /**
         * @brief Board state enumeration
         */
        enum class State : uint8_t {
            DEINIT = 0,    /*!< Board is not initialized */
            INIT,          /*!< Board is initialized */
            BEGIN,         /*!< Board is started */
        };

        /**
         * @brief Default constructor, initializes the board with default configuration.
         *
         * If no default configuration is provided, the error message will be printed when `init()` or `begin()` is called.
         * There are three ways to provide a default configuration:
         * 1. Use the `esp_panel_board_supported_conf.h` file to enable a supported board
         * 2. Use the `esp_panel_board_custom_conf.h` file to define a custom board
         * 3. Use menuconfig to enable a supported board or define a custom board
         */
        Board();

        /**
         * @brief Constructor with configuration
         *
         * @param[in] config Board configuration structure
         */
        Board(const BoardConfig &config): _config(config) {}

        /**
         * @brief Destructor
         */
        ~Board();

        /**
         * @brief Configure the callback function for a specific stage
         *
         * @param[in] type Callback type
         * @param[in] callback Callback function
         * @return `true` if successful, `false` otherwise
         */
        bool configCallback(board::BoardConfig::StageCallbackType type, BoardConfig::FunctionStageCallback callback);

        /**
         * @brief Initialize the panel device
         *
         * Creates objects for the LCD and Backlight devices based on the configuration.
         *
         * @return `true` if successful, `false` otherwise
         */
        bool init();

        /**
         * @brief Startup the panel device
         *
         * Initializes and configures all enabled devices in the following order: `LCD -> Backlight`
         *
         * @return `true` if successful, `false` otherwise
         * @note Will automatically call `init()` if not already initialized
         */
        bool begin();

        /**
         * @brief Delete the panel device and release resources
         *
         * Releases all device instances in the following order: `Backlight -> LCD`
         *
         * @return `true` if successful, `false` otherwise
         * @note After calling this function, the board returns to uninitialized state
         */
        bool del();

        /**
         * @brief Check if current state is greater than or equal to given state
         *
         * @param[in] state State to compare with
         * @return `true` if current state is greater than or equal to given state
         */
        bool isOverState(State state)
        {
            return (this->_state >= state);
        }

        /**
         * @brief Get the LCD driver instance
         *
         * @return Pointer to the LCD driver instance, or `nullptr` if LCD is not enabled or not initialized
         */
        drivers::LCD *getLCD()
        {
            return _lcd_device.get();
        }

        /**
         * @brief Get the Backlight driver instance
         *
         * @return Pointer to the Backlight driver instance, or `nullptr` if Backlight is not enabled or not initialized
         */
        drivers::Backlight *getBacklight()
        {
            return _backlight.get();
        }

        /**
         * @brief Get the current board configuration
         *
         * @return Reference to the current board configuration
         */
        const BoardConfig &getConfig() const
        {
            return _config;
        }

        /**
         * @brief Alias for backward compatibility
         * @deprecated Use `getLCD()` instead
         */
        [[deprecated("Use `getLCD()` instead")]]
        drivers::LCD *getLcd()
        {
            return getLCD();
        }

        /**
         * @brief Alias for backward compatibility
         * @deprecated Use `getLCD()->getFrameWidth()` instead
         */
        [[deprecated("Use `getLCD()->getFrameWidth()` instead")]]
        int getLcdWidth()
        {
            return (getLCD() != nullptr) ? getLCD()->getFrameWidth() : -1;
        }

        /**
         * @brief Alias for backward compatibility
         * @deprecated Use `getLCD()->getFrameHeight()` instead
         */
        [[deprecated("Use `getLCD()->getFrameHeight()` instead")]]
        int getLcdHeight()
        {
            return (getLCD() != nullptr) ? getLCD()->getFrameHeight() : -1;
        }

    private:
        /**
         * @brief Set the current board state
         *
         * @param[in] state New state to set
         */
        void setState(State state)
        {
            _state = state;
        }

        /**
         * @brief Check if LCD is used
         *
         * @return `true` if LCD is used, `false` otherwise
         */
        bool isLCD_Used()
        {
            return _config.lcd.has_value();
        }

        /**
         * @brief Check if Backlight is used
         *
         * @return `true` if Backlight is used, `false` otherwise
         */
        bool isBacklightUsed()
        {
            return _config.backlight.has_value();
        }

        BoardConfig _config = {};
        bool _use_default_config = false;
        State _state = State::DEINIT;
        std::shared_ptr<drivers::Bus> _lcd_bus = nullptr;
        std::shared_ptr<drivers::LCD> _lcd_device = nullptr;
        std::shared_ptr<drivers::Backlight> _backlight = nullptr;
    };

} // namespace esp_panel

/**
 * @brief Alias for backward compatibility
 * @deprecated Use `esp_panel::board::Board` instead
 */
using ESP_Panel [[deprecated("Use `esp_panel::board::Board` instead")]] = esp_panel::board::Board;
