/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <unordered_map>
#include <memory>
#include <string>
#include <variant>
#include "soc/soc_caps.h"
#include "utils/esp_panel_utils_cxx.hpp"
#include "esp_panel_bus_conf_internal.h"
#include "esp_panel_bus.hpp"
#include "esp_panel_bus_dsi.hpp"

namespace esp_panel::drivers {

    /**
     * @brief The bus factory class for creating and managing bus devices
     *
     * This class provides static methods to create bus devices and manage their configurations
     */
    class BusFactory {
    public:
        /**
         * @brief The bus configuration variant type
         *
         * Contains configuration for MIPI DSI bus only
         */
        using Config = std::variant<BusDSI::Config>;

        /**
         * @brief Function pointer type for bus device constructors
         *
         * Points to functions that create and return a shared pointer to a bus device
         */
        using FunctionDeviceConstructor = std::shared_ptr<Bus> (*)(const Config &config);

        /**
         * @brief Create a new bus device with configuration
         *
         * @param[in] config The bus configuration
         *
         * @return Shared pointer to the device if successful, `nullptr` otherwise
         */
        static std::shared_ptr<Bus> create(const Config &config);

        /**
         * @brief Get the bus type from configuration
         *
         * @param[in] config The bus configuration
         *
         * @return Bus type `ESP_PANEL_BUS_TYPE_*`) if successful, `-1` otherwise
         */
        static int getConfigType(const Config &config);

        /**
         * @brief Get the string representation of a bus type
         *
         * @param[in] type The bus type `ESP_PANEL_BUS_TYPE_*`)
         *
         * @return Bus type name string if successful, `"Unknown"` otherwise
         */
        static utils::string getTypeNameString(int type);

    private:
        /**
         * @brief Map of bus types to their type names
         *
         * Maps each bus type to its type name string
         */
        static const utils::unordered_map<int, utils::string> _type_name_map;

        /**
         * @brief Map of bus types to their constructors and type names
         *
         * Maps each bus type to its constructor function
         */
        static const utils::unordered_map<int, FunctionDeviceConstructor> _type_constructor_map;
    };

} // namespace esp_panel::drivers
