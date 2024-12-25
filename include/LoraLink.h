#pragma once

#include <Arduino.h>
#include <functional>

namespace LoraLink
{
    using OnCommandCallback = std::function<void(float, float)>; // linear, angular

    /**
     * @brief Initialize the LoraLink UART and start the command reading task
     *
     * @param ser The serial port to use
     */
    void begin(HardwareSerial &ser);

    /**
     * @brief Set the callback funtion to call when a LoraLink command is received
     *
     * It's a good idea to set this function before calling `begin()`
     *
     * @param callback The callback function
     */
    void setOnCommandCallback(OnCommandCallback callback);

    /**
     * @brief Set the callback function to call when the LoraLink connection is lost
     *
     *  It's a good idea to set this function before calling `begin()`
     * 
     * @param callback The callback function
     */
    void setOnLinkLostCallback(callback_function_t callback);
} // namespace LoraLink
