#pragma once

#include <Arduino.h>

namespace SensorCollector
{
    void begin();
    void registerSensorCb(String name, callback_function_t readFunc);

} // namespace SensorCollector
