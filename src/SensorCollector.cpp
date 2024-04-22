#include "SensorCollector.hpp"
#include <STM32FreeRTOS.h>
#include <timers.h>
#include "Logger.hpp"

namespace SensorCollector
{
    constexpr uint8_t MAX_SENSOR_COUNT = 10;
    static void (*sensorCallbacks[MAX_SENSOR_COUNT])(void){nullptr};
    static uint8_t sensorCount = 0;

    void collectData(TimerHandle_t)
    {
        for (auto i = 0; i < sensorCount; i++)
        {
            sensorCallbacks[i]();
        }
    }

    void begin()
    {
        auto sensor_handle = xTimerCreate("Sensor Read", 5, pdTRUE, nullptr, collectData);
        xTimerStart(sensor_handle, 0);
    }

    void registerSensorCb(String name, void (*readFunc)(void))
    {
        if (sensorCount < MAX_SENSOR_COUNT)
        {
            sensorCallbacks[sensorCount++] = readFunc;
        }
        else
        {
            Logger::error("Unable to register " + name + ", max sensor count reached");
        }
    }
} // namespace sensors
