#pragma once

#include <Arduino.h>
#include <memory>
#include <STM32FreeRTOS.h>
#include <timers.h>

#include "SensorCollector.h"
#include "config.h"

class Motor
{
private:
    uint8_t enablePin;
    uint8_t in1Pin;
    uint8_t in2Pin;

    std::shared_ptr<HardwareTimer> encoderTimer;

    // bool _direction = 0;                      // 0 for forward, 1 for backward
    int32_t targetFreq = 0;                   // Target encoder frequency in Hz, negative for backward
    static constexpr uint8_t PID_PERIOD = 50; // Fuck PID (and measure speed) every 50ms
    static constexpr uint8_t MIN_DUTY = 10;
    static constexpr uint8_t MAX_DUTY = 200;       // To avoid Flying away
    static constexpr uint8_t MIN_TARGET_FREQ = 30; // Minimum rotating speed, lower speed may result in inaccurate measurement

    float PID_KP = 0.3;
    float PID_KI = 0.2;
    float PID_KD = 0.5;

    static constexpr uint32_t SPEED_SCALE = 1404 / (PI * 0.125); // Linear speed(m/s) to encoder output frequency(Hz)，1404 pulses/round, 0.125m diameter
    static constexpr uint32_t MEASURE_SPEED_PERIOD = 100;        // Measure speed every 100ms

    volatile int32_t freqMeasured; // Measured encoder freqency, negative for backward

    /**
     * @brief Set the duty cycle of the PWM signal
     *
     * @param duty Duty cycle between 0 and 255
     */
    inline void setDuty(uint8_t duty)
    {
        analogWrite(enablePin, duty);
    }

    /**
     * @brief Get the encoder input frequency
     *
     * @return The frequency measured
     */
    inline uint32_t getInputFrequency()
    {
        return freqMeasured;
    }

public:
    Motor(uint8_t in1_pin,
          uint8_t in2_pin,
          uint8_t enable_pin,
          EncoderCfg encoder);

    ~Motor();

    /**
     * @brief Set the direction signal
     *
     * @param direction True for backward, false for forward
     */
    inline void setDirection(bool direction)
    {
        // _direction = direction;
        digitalWrite(in1Pin, direction);
        digitalWrite(in2Pin, !direction);
    }

    /**
     * @brief Stop the motor
     *
     * @param hard Whether to stop the motor hard
     */
    inline void stop(bool hard = false)
    {
        digitalWrite(in1Pin, hard);
        digitalWrite(in2Pin, hard);
    }

    /**
     * @brief Set the target speed of the motor
     *
     * @param speed Target speed in m/s, positive for forward, negative for backward
     */
    void setSpeed(float speed);

    /**
     * @brief Set the PID controller arguments
     *
     * @param kp: Proportional gain
     * @param ki: Integral gain
     * @param kd: Derivative gain
     */
    inline void setPID(float kp, float ki, float kd)
    {
        PID_KP = kp;
        PID_KI = ki;
        PID_KD = kd;
    }

    /**
     * @brief Get the current speed of the motor
     *
     * @return The current speed in m/s
     */
    inline float getSpeed()
    {
        return static_cast<float>(freqMeasured) / SPEED_SCALE;
    }

    /**
     * @brief The PID implementation function
     *
     * Used in an OS timer, don't call it manually
     */
    friend void fuckPID(TimerHandle_t);
};
