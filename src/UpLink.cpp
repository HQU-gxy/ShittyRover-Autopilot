#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <timers.h>
#include <ulog.h>

#include "UpLink.h"
#include "config.h"

namespace UpLink
{
  static bool avail;

  static onUpLinkCommandCB onCmdCB;
  static getStatusFunc getStatus;

  constexpr uint8_t SEND_STATUS_PERIOD = 50; // ms
  constexpr uint8_t CHECK_CMD_PERIOD = 20;   // ms

  struct __attribute__((packed)) UpLinkCommand
  {
    uint8_t header;      // Should be 0x7b
    float targetLinear;  // Linear speed in m/s
    float targetAngular; // Angular speed in rad/s
    uint8_t checksum;
  };

  void sendStat(TimerHandle_t)
  {
    if (!getStatus)
    {
      ULOG_WARNING("Get status function not set, will not send status");
      return;
    }

    auto [currLinear, currAngular, imuData] = getStatus();

    struct __attribute__((packed))
    {
      uint8_t header = 0x69;
      float currentLinear;  // Linear speed in m/s
      float currentAngular; // Angular speed in rad/s
      // Accel in m/s^2
      IMU::IMUData imu;
      uint8_t checksum;
    } status{
        .currentLinear = currLinear,
        .currentAngular = currAngular,
        .imu = imuData,
    };
    auto bytes = reinterpret_cast<uint8_t *>(&status);
    uint8_t sum;
    for (uint8_t i = 0; i < sizeof(status) - 1; i++)
    {
      sum ^= bytes[i];
    }
    status.checksum = sum;
    Serial3.write(bytes, sizeof(status));
  }

  void readCmdTask(void *)
  {
    while (1)
    {
      vTaskDelay(pdMS_TO_TICKS(CHECK_CMD_PERIOD));

      if (!Serial3.available())
        continue;

      if (Serial3.peek() != 0x7b) // The header byte
      {
        Serial3.read();
        continue;
      }

      char buf[16];
      Serial3.readBytes(buf, sizeof(UpLinkCommand));
      auto parsed = reinterpret_cast<const UpLinkCommand *>(buf);
      uint8_t sum = 0;
      for (uint8_t i = 0; i < sizeof(UpLinkCommand) - 1; i++)
      {
        sum ^= buf[i];
      }

      if (sum != parsed->checksum)
      {
        ULOG_ERROR("UpLink command checksum error: %x", sum);
        continue;
      }
      // targetSpeed.first = parsed->targetLinear;
      // targetSpeed.second = parsed->targetAngular;
      if (onCmdCB)
        onCmdCB(parsed->targetLinear, parsed->targetAngular);
      else
        ULOG_WARNING("No callback set for UpLink command");
    }
  }

  void begin()
  {
    Serial3.setTx(UART1_TX_PIN);
    Serial3.setRx(UART1_RX_PIN);
    Serial3.setTimeout(20);
    Serial3.begin(115200);
    auto sendStatTimer = xTimerCreate("Send status", pdMS_TO_TICKS(SEND_STATUS_PERIOD), true, (void *)69, sendStat);
    xTimerStart(sendStatTimer, 0);
    xTaskCreate(readCmdTask, "Read command", 1024, nullptr, osPriorityAboveNormal, nullptr);
  }

  void setOnCmdCallback(onUpLinkCommandCB cb)
  {
    onCmdCB = cb;
  }

  void setGetStatusFunc(getStatusFunc func)
  {
    getStatus = func;
  }

} // namespace UpLink