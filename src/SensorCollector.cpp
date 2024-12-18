#include <STM32FreeRTOS.h>
#include <timers.h>
#include <ulog.h>
#include <vector>

#include "SensorCollector.h"

namespace SensorCollector
{
    constexpr uint8_t MAX_SENSOR_COUNT = 10;
    constexpr uint8_t SENSOR_READ_PERIOD = 10; // The period in ms to read the sensors
    static std::vector<callback_function_t> sensorCallbacks;
    static uint8_t sensorCount = 0;

    void collectData(TimerHandle_t)
    {
        for (auto &cb : sensorCallbacks)
        {
            cb();
        }
    }

    void begin()
    {
        auto sensor_handle = xTimerCreate("Sensor Read", SENSOR_READ_PERIOD, pdTRUE, nullptr, collectData);
        xTimerStart(sensor_handle, 0);
    }

    void registerSensorCb(String name, callback_function_t readFunc)
    {
        if (!readFunc)
        {
            ULOG_ERROR("Sensor callback function is null");
            return;
        }
        if (sensorCount < MAX_SENSOR_COUNT)
        {
            sensorCallbacks.push_back(readFunc);
            sensorCount++;
        }
        else
        {
            ULOG_ERROR(("Unable to register " + name + ", max sensor count reached").c_str());
        }
    }
} // namespace sensors
