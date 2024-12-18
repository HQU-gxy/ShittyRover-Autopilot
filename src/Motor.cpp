#include <ulog.h>
#include <STM32FreeRTOS.h>
#include <stm32g431xx.h>

#include "Motor.h"

void fuckPID(TimerHandle_t shit)
{

  auto motor = static_cast<Motor *>(pvTimerGetTimerID(shit));
  if (!motor)
  {
    ULOG_ERROR("Fuck PID is called without a Motor pointer passed in");
    return;
  }

  auto currentCounter = static_cast<int16_t>(LL_TIM_GetCounter(motor->encoderTimer->getHandle()->Instance) & 0xffff);
  motor->freqMeasured = currentCounter * (1000 / Motor::MEASURE_SPEED_PERIOD);
  LL_TIM_SetCounter(motor->encoderTimer->getHandle()->Instance, 0);

  // An incredibly shitty PID implementation
  static int32_t lastError = 0;
  static int32_t lastLastError = 0;
  static int32_t lastOutput = 0;

  int32_t currentError = motor->targetFreq - motor->freqMeasured;
  int32_t output = lastOutput + motor->PID_KP * (currentError - lastError) + motor->PID_KI * currentError + motor->PID_KD * (currentError - 2 * lastError + lastLastError);

  if (output < motor->MIN_DUTY)
    output = motor->MIN_DUTY;
  else if (output > motor->MAX_DUTY)
    output = motor->MAX_DUTY;

  if (motor->targetFreq == 0)
    output = 0;
  motor->setDuty(output);

  lastOutput = output;
  lastError = currentError;
  lastLastError = lastError;

#ifdef PID_TUNING
  struct __attribute__((packed))
  {
    uint8_t header = 0x69;
    uint32_t freq;
    int32_t error;
    int32_t output;
    char shit[4] = {'s', 'h', 'i', 't'};
  } pidStatus{
      .freq = motor->freqMeasured,
      .error = currentError,
      .output = output,
  };
  Serial2.write(reinterpret_cast<char *>(&pidStatus), sizeof(pidStatus));
#endif
}

Motor::Motor(uint8_t in1_pin,
             uint8_t in2_pin,
             uint8_t enable_pin,
             EncoderCfg encoder)
    : in1Pin(in1_pin),
      in2Pin(in2_pin),
      enablePin(enable_pin)
{
  pinMode(in1Pin, INPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  pinmap_pinout(digitalPinToPinName(encoder.encA_Pin), PinMap_TIM);
  pinmap_pinout(digitalPinToPinName(encoder.encB_Pin), PinMap_TIM);

  encoderTimer = std::make_shared<HardwareTimer>(encoder.timer);
  encoderTimer->setPreloadEnable(false);

  LL_TIM_ENCODER_InitTypeDef encInitStruct{
      .EncoderMode = LL_TIM_ENCODERMODE_X4_TI12,
      .IC1Polarity = LL_TIM_IC_POLARITY_RISING,
      .IC1ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI,
      .IC1Prescaler = LL_TIM_ICPSC_DIV4,
      .IC1Filter = LL_TIM_IC_FILTER_FDIV1,
      .IC2Polarity = LL_TIM_IC_POLARITY_RISING,
      .IC2ActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI,
      .IC2Prescaler = LL_TIM_ICPSC_DIV4,
      .IC2Filter = LL_TIM_IC_FILTER_FDIV1,
  };
  LL_TIM_ENCODER_Init(encoder.timer, &encInitStruct);
  LL_TIM_DisableMasterSlaveMode(encoder.timer);
  LL_TIM_SetTriggerOutput(encoder.timer, LL_TIM_TRGO_RESET);
  LL_TIM_SetTriggerOutput2(encoder.timer, LL_TIM_TRGO2_RESET);
  LL_TIM_SetCounter(encoder.timer, 0);
  LL_TIM_EnableCounter(encoder.timer);

  auto pidTimer = xTimerCreate("PID", pdMS_TO_TICKS(PID_PERIOD), pdTRUE, static_cast<void *>(this), fuckPID);

  xTimerStart(pidTimer, 0);
  ULOG_DEBUG("PID Timer started");
}

void Motor::setSpeed(float speed)
{
  if (speed < 0)
  {
    setDirection(1);
    speed = -speed;
  }
  else
  {
    setDirection(0);
  }

  targetFreq = speed * SPEED_SCALE;
  if (targetFreq < MIN_TARGET_FREQ) // Wether to set the motor to stop
  {
    if (targetFreq < MIN_TARGET_FREQ / 2)
      targetFreq = 0;
    else
      targetFreq = MIN_TARGET_FREQ;
  }
}

void Motor::beep(uint16_t f, uint16_t t)
{
  analogWrite(enablePin, MCPWM_BEEP_DC * 255 / 100);

  auto startTime = millis();

  uint16_t halfPeriod = 1e6 / (2 * f); // In μs

  while (millis() - startTime < t)
  {
    setDirection(0);
    delayMicroseconds(halfPeriod);
    setDirection(1);
    delayMicroseconds(halfPeriod);
  }

  stop();
}

Motor::~Motor()
{
}
