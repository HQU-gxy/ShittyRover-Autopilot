#pragma once

#include <Arduino.h>
#include <memory>
#include <STM32FreeRTOS.h>
#include <timers.h>

#include "SensorCollector.h"

class Motor
{
private:
    uint8_t enablePin;
    uint8_t in1Pin;
    uint8_t in2Pin;

    std::shared_ptr<HardwareTimer> encoderTimer;

    bool _direction = 0;                      // 0 for forward, 1 for backward
    uint32_t targetFreq = 0;                  // Target encoder frequency in Hz
    static constexpr uint8_t PID_PERIOD = 50; // Fuck PID every 50ms
    static constexpr uint8_t MIN_DUTY = 10;
    static constexpr uint8_t MAX_DUTY = 200;       // To avoid Flying away
    static constexpr uint8_t MIN_TARGET_FREQ = 15; // Whether to park

    float PID_KP = 0.3;
    float PID_KI = 0.2;
    float PID_KD = 0.5;

    static constexpr uint32_t SPEED_SCALE = 56; // Linear speed(m/s) to encoder output frequency(Hz)

    volatile uint32_t freqMeasured, lastCapture = 0, currentCapture;
    uint32_t rolloverCompareCount = 0;

    /**
     * Set the duty cycle of the PWM signal
     * @param duty: duty cycle between 0 and 255
     */
    inline void setDuty(uint8_t duty)
    {
        analogWrite(enablePin, duty);
    }

    /**
     * Get the encoder input frequency
     * @return the frequency measured
     */
    inline uint32_t getInputFrequency()
    {
        return freqMeasured;
    }

public:
    Motor(uint8_t in1_pin,
          uint8_t in2_pin,
          uint8_t enable_pin,
          TIM_TypeDef *encoder_timer);

    ~Motor();

    /**
     * Set the direction signal
     * @param direction: direction signal value
     */
    inline void setDirection(bool direction)
    {
        _direction = direction;
        digitalWrite(in1Pin, direction);
        digitalWrite(in2Pin, !direction);
    }

    /**
     * Set the target speed of the motor
     * @param speed: target speed in m/s, positive for forward, negative for backward
     */
    void setSpeed(float speed);

    /**
     * Set the PID controller arguments
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
     * The PID implementation function
     *
     * Used in an OS timer, don't call it manually
     */
    friend void fuckPID(TimerHandle_t);
};
