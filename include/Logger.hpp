#pragma once

#include <Arduino.h>

namespace Logger
{
    void begin();

    void trace(String msg);
    void debug(String msg);
    void info(String msg);
    void warn(String msg);
    void error(String msg);
    void critical(String msg);

}