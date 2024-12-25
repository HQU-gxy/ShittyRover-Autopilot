#include "LoraLink.h"
#include <STM32FreeRTOS.h>
#include <timers.h>
#include <ulog.h>
#include "config.h"

namespace LoraLink
{
    constexpr uint8_t CHECK_CMD_PERIOD = 50; // ms

    OnCommandCallback onCommandCallback;
    callback_function_t onLinkLostCallback;

    HardwareSerial *LoraSerial;

    struct __attribute__((packed)) LoraLinkCommand
    {
        uint8_t header;      // Should be 0x12
        float targetLinear;  // Linear speed in m/s
        float targetAngular; // Angular speed in rad/s
        uint8_t checksum;
    };

    void checkForCommandTask(void *)
    {
        constexpr uint8_t MAX_RETRY = 10;
        uint8_t retryCount = 0;
        auto startTime = xTaskGetTickCount();
        while (1)
        {
            vTaskDelayUntil(&startTime, pdMS_TO_TICKS(CHECK_CMD_PERIOD));
            if (retryCount++ == MAX_RETRY)
            {
                if (onLinkLostCallback)
                {
                    onLinkLostCallback();
                }
                else
                {
                    ULOG_WARNING("LoraLink: No on-link-lost callback set");
                }
                retryCount = 0;
            }

            if (!LoraSerial->available())
                continue;

            if (LoraSerial->peek() != 0x12) // The header byte
            {
                LoraSerial->read();
                continue;
            }

            if (!onCommandCallback)
            {
                ULOG_WARNING("LoraLink: No on-command callback set");
                return;
            }

            char buf[sizeof(LoraLinkCommand)];
            LoraSerial->readBytes(buf, sizeof(LoraLinkCommand));
            auto parsed = reinterpret_cast<const LoraLinkCommand *>(buf);
            uint8_t sum = 0;
            for (uint8_t i = 0; i < sizeof(LoraLinkCommand) - 1; i++)
            {
                sum ^= buf[i];
            }

            if (sum != parsed->checksum)
            {
                ULOG_ERROR("UpLink command checksum error: %x", sum);
                continue;
            }

            onCommandCallback(parsed->targetLinear, parsed->targetAngular);
            retryCount = 0;
        }
    }

    void begin(HardwareSerial &ser)
    {
        LoraSerial = &ser;
        LoraSerial->setTx(UART1_TX_PIN);
        LoraSerial->setRx(UART1_RX_PIN);
        LoraSerial->begin(38400);
        LoraSerial->setTimeout(20);
        
        xTaskCreate(checkForCommandTask, "LoraLinkCheckForCommand", 512, nullptr, osPriorityAboveNormal2, nullptr);
    }

    void setOnCommandCallback(OnCommandCallback callback)
    {
        onCommandCallback = callback;
    }

    void setOnLinkLostCallback(callback_function_t callback)
    {
        onLinkLostCallback = callback;
    }
}