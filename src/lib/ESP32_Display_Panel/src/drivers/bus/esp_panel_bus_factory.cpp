/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "soc/soc_caps.h"
#include "utils/esp_panel_utils_log.h"
#include "esp_utils_helpers.h"
#include "esp_panel_bus_conf_internal.h"
#include "esp_panel_bus_factory.hpp"

namespace esp_panel::drivers {

    #define TYPE_NAME_MAP_ITEM(type_name)                                                 \
    {                                                                               \
        Bus ##type_name::BASIC_ATTRIBUTES_DEFAULT.type, #type_name   \
    }

    #define DEVICE_CREATOR(type_name)                                                                     \
    [](const BusFactory::Config &config) -> std::shared_ptr<Bus> {                                                            \
        ESP_UTILS_LOG_TRACE_ENTER();                                                                  \
        std::shared_ptr<Bus> device = nullptr;                                                        \
        ESP_UTILS_CHECK_FALSE_RETURN(std::holds_alternative<Bus ##type_name::Config>(config), device, \
        "Bus config is not a " #type_name " config"                                               \
        );                                                                                            \
        ESP_UTILS_CHECK_EXCEPTION_RETURN(                                                             \
        (device = utils::make_shared<Bus ##type_name>(                                        \
        std::get<Bus ##type_name::Config>(config))                                            \
        ), nullptr, "Create " #type_name " failed"                                                \
        );                                                                                            \
        ESP_UTILS_LOG_TRACE_EXIT();                                                                   \
        return device;                                                                                \
    }

    #define TYPE_CREATOR_MAP_ITEM(type_name)                                                 \
    {                                                                               \
        Bus ##type_name::BASIC_ATTRIBUTES_DEFAULT.type, DEVICE_CREATOR(type_name)   \
    }

    const utils::unordered_map<int, utils::string> BusFactory::_type_name_map = {
        TYPE_NAME_MAP_ITEM(DSI),
    };

    const utils::unordered_map<int, BusFactory::FunctionDeviceConstructor> BusFactory::_type_constructor_map = {
        TYPE_CREATOR_MAP_ITEM(DSI),
    };

    std::shared_ptr<Bus> BusFactory::create(const Config &config)
    {
        ESP_UTILS_LOG_TRACE_ENTER();
        ESP_UTILS_LOGD("Param: config(@%p)", &config);

        auto name = getTypeNameString(getConfigType(config)).c_str();
        auto type = getConfigType(config);
        ESP_UTILS_LOGD("Get config type: %d(%s)", type, name);

        auto it = _type_constructor_map.find(type);
        ESP_UTILS_CHECK_FALSE_RETURN(
            it != _type_constructor_map.end(), nullptr, "Disabled or unsupported type: %d(%s)", type, name
        );

        std::shared_ptr<Bus> device = it->second(config);
        ESP_UTILS_CHECK_NULL_RETURN(device, nullptr, "Create device(%s) failed", name);

        ESP_UTILS_LOG_TRACE_EXIT();
        return device;
    }

    int BusFactory::getConfigType(const Config &config)
    {
        if (std::holds_alternative<BusDSI::Config>(config)) {
            return BusDSI::BASIC_ATTRIBUTES_DEFAULT.type;
        }
        return -1;
    }

    utils::string BusFactory::getTypeNameString(int type)
    {
        auto it = _type_name_map.find(type);
        if (it != _type_name_map.end()) {
            return it->second;
        }
        return "Unknown";
    }

} // namespace esp_panel::drivers
