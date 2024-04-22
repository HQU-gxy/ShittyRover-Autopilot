#include <ulog.h>
#include <STM32FreeRTOS.h>
#include <semphr.h>

#include "Logger.hpp"
#include "config.h"

namespace Logger
{
    static xSemaphoreHandle loggerMutex;
    void my_console_logger(ulog_level_t severity, char *msg)
    {
        if (xSemaphoreTake(loggerMutex, 10) == pdTRUE)
        {
            Serial2.printf("%d [%s]: %s\n", millis(), ulog_level_name(severity), msg);
            xSemaphoreGive(loggerMutex);
        }
    }

    void begin()
    {
        Serial2.setTx(UART2_TX);
        Serial2.setRx(UART2_RX);
        Serial2.begin(115200);
        loggerMutex = xSemaphoreCreateMutex();

        ulog_init();
        ulog_subscribe(my_console_logger, ULOG_DEBUG_LEVEL);
    }

    void trace(String msg)
    {
        ulog_message(ULOG_TRACE_LEVEL, msg.c_str());
    }

    void debug(String msg)
    {
        ulog_message(ULOG_DEBUG_LEVEL, msg.c_str());
    }

    void info(String msg)
    {
        ulog_message(ULOG_INFO_LEVEL, msg.c_str());
    }

    void warn(String msg)
    {
        ulog_message(ULOG_WARNING_LEVEL, msg.c_str());
    }

    void error(String msg)
    {
        ulog_message(ULOG_ERROR_LEVEL, msg.c_str());
    }

    void critical(String msg)
    {
        ulog_message(ULOG_CRITICAL_LEVEL, msg.c_str());
    }

} // namespace Logger
