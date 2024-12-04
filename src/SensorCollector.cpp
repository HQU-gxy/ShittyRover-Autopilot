#include "SensorCollector.hpp"
#include <STM32FreeRTOS.h>
#include <timers.h>
#include <ulog.h>

namespace SensorCollector
{
    constexpr uint8_t MAX_SENSOR_COUNT = 10;
    static callback_function_t sensorCallbacks[MAX_SENSOR_COUNT]{nullptr};
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

    void registerSensorCb(String name, callback_function_t readFunc)
    {
        if (sensorCount < MAX_SENSOR_COUNT)
        {
            sensorCallbacks[sensorCount++] = readFunc;
        }
        else
        {
            ULOG_ERROR(("Unable to register " + name + ", max sensor count reached").c_str());
        }
    }
} // namespace sensors
