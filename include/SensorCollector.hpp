#pragma once

#include <Arduino.h>

namespace SensorCollector
{
    void begin();
    void registerSensorCb(String name, void(readFunc)(void));

} // namespace SensorCollector
