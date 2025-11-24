/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "utils/esp_panel_utils_log.h"
#include "esp_utils_helpers.h"
#include "esp_panel_lcd_conf_internal.h"
#include "esp_panel_lcd_factory.hpp"

namespace esp_panel::drivers {

    #define DEVICE_CREATOR(controller) \
    [](const BusFactory::Config &bus_config, const LCD::Config &lcd_config) -> std::shared_ptr<LCD> { \
        std::shared_ptr<LCD> device = nullptr; \
        ESP_UTILS_CHECK_EXCEPTION_RETURN( \
        (device = utils::make_shared<LCD_ ## controller>(bus_config, lcd_config)), nullptr, \
        "Create " #controller " failed" \
        ); \
        return device; \
    }

    #define MAP_ITEM(controller) \
    {LCD_ ##controller::BASIC_ATTRIBUTES_DEFAULT.name, DEVICE_CREATOR(controller)}

    const utils::unordered_map<utils::string, LCD_Factory::FunctionDeviceConstructor> LCD_Factory::_name_function_map = {
        MAP_ITEM(EK79007),
    };

    std::shared_ptr<LCD> LCD_Factory::create(
        utils::string name, const BusFactory::Config &bus_config, const LCD::Config &lcd_config
    )
    {
        ESP_UTILS_LOGD("Param: name(%s), bus_config(@%p), lcd_config(@%p)", name.c_str(), &bus_config, &lcd_config);

        auto it = _name_function_map.find(name);
        ESP_UTILS_CHECK_FALSE_RETURN(it != _name_function_map.end(), nullptr, "Unknown controller: %s", name.c_str());

        std::shared_ptr<LCD> device = it->second(bus_config, lcd_config);
        ESP_UTILS_CHECK_NULL_RETURN(device, nullptr, "Create device failed");

        return device;
    }

} // namespace esp_panel::drivers
