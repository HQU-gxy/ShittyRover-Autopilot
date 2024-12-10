#pragma once

#include <Arduino.h>

// LED pins
constexpr uint8_t LED1_PIN = PC2;
constexpr uint8_t LED2_PIN = PC3;

// Button pins
constexpr uint8_t BUTTON1_PIN = PA12;
constexpr uint8_t BUTTON2_PIN = PA11;
constexpr uint8_t BUTTON3_PIN = PA10;

// IMU pins
constexpr uint8_t ACCEL_CS_PIN = PB1;
constexpr uint8_t GYRO_CS_PIN = PB0;
constexpr uint8_t SPI1_SCK_PIN = PA5;
constexpr uint8_t SPI1_MISO_PIN = PA6;
constexpr uint8_t SPI1_MOSI_PIN = PA7;

// Compass pins
constexpr uint8_t I2C1_SDA_PIN = PB9;
constexpr uint8_t I2C1_SCL_PIN = PA15;

// Compass magnetic declination
constexpr float MAGNETIC_DECLINATION = -(4.0 + (38.0 / 60.0)) / (180 / PI);

// UART pins
// Reserve for future peripherals
constexpr uint8_t UART1_TX_PIN = PB7;
constexpr uint8_t UART1_RX_PIN = PB6;
// For debugging
constexpr uint8_t UART2_TX_PIN = PA2;
constexpr uint8_t UART2_RX_PIN = PA3;
// For uplink
constexpr uint8_t UART3_TX_PIN = PB10;
constexpr uint8_t UART3_RX_PIN = PB11;

// Power monitor pins
constexpr uint8_t ISENS_PIN = PA0;
constexpr uint8_t VSENS_PIN = PA1;

// Motor pins
constexpr uint8_t MOTOR1_EN_PIN = PB4;
constexpr uint8_t MOTOR1_IN1_PIN = PC11;
constexpr uint8_t MOTOR1_IN2_PIN = PC12;
constexpr uint16_t MOTOR1_ENC_A_PIN = PC1; // Pay attention to the AF number
constexpr uint16_t MOTOR1_ENC_B_PIN = PC0; // Pay attention to the AF number
#define MOTOR1_ENC_TIMER TIM1

constexpr uint8_t MOTOR2_EN_PIN = PB5;
constexpr uint8_t MOTOR2_IN1_PIN = PD2;
constexpr uint8_t MOTOR2_IN2_PIN = PB3;
constexpr uint16_t MOTOR2_ENC_A_PIN = PC7; // Pay attention to the AF number
constexpr uint16_t MOTOR2_ENC_B_PIN = PC6; // Pay attention to the AF number
#define MOTOR2_ENC_TIMER TIM3

constexpr uint16_t MCPWM_BEEP_DC = 20;