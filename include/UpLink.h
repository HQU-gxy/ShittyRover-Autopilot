#pragma once

#include <utility>
#include <functional>

#include "IMU.h"

namespace UpLink
{
    using onUpLinkCommandCB = std::function<void(const float targetLinear, const float targetAngular)>;
    // Current linear speed, current angular speed, IMU data
    using getStatusFunc = std::function<std::tuple<float, float, IMU::IMUData>()>;

    /**
     * @brief Initialize the UpLink UART and start the command reading task and status sending timer
     */
    void begin();

    /**
     * @brief Set the callback funtion to call when an uplink command is fucking over
     *
     * It's a good idea to set this function before calling `begin()`
     *
     * @param
     */
    void setOnCmdCallback(onUpLinkCommandCB cb);

    /**
     * @brief Set the function to get the current status of the car
     *
     * It's a good idea to set this function before calling `begin()`
     *
     * @param func The function to get the current status of the car
     */
    void setGetStatusFunc(getStatusFunc func);

} // namespace UpLink