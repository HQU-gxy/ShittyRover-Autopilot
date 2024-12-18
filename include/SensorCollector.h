#pragma once

#include <Arduino.h>

/**
 * @brief SensorCollector
 *
 * This namespace provides a way to collect data from multiple sensors
 *
 */

namespace SensorCollector
{
    /**
     * @brief Initialize the sensor collector
     */
    void begin();

    /**
     * @brief Register a sensor callback function
     *
     * @param name The name of the sensor, used only for debugging
     * @param readFunc The callback function to read the sensor data
     */
    void registerSensorCb(String name, callback_function_t readFunc);

} // namespace SensorCollector
