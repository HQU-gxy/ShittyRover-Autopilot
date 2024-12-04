#include <UpLink.h>

/*
    This module is for communicating with the IO_MCU which controls the motors.
    Following is the request packet structure:
        1 byte: 0xAA -> packet head
        1 byte: 0x00 - 0xFF -> msg type
        1 byte: 0x01 - 0x20 -> payload size, 32 bytes max
        ${payload size} bytes: -> payload
        1 byte: 0x00 - 0xFF -> payload checksum

    Following is the respons packet structure:
        1 byte: 0x55 -> packet head
        1 byte: 0x00/0x01 -> NACK/ACK
        1 byte: 0x00 - 0x20 -> payload size, 32 bytes max
        ${payload size} bytes: -> payload
        1 byte: 0x00 - 0xFF -> payload checksum
*/

namespace RoverIO
{

    constexpr uint8_t REQUEST_HEAD = 0xaa;
    constexpr uint8_t RESPONSE_HEAD = 0x55;

    uint8_t checksum(const uint8_t *data, uint8_t len)
    {
        uint8_t checksum = 0;
        for (uint8_t i = 0; i < len; i++)
        {
            checksum += data[i];
        }
        return checksum;
    }

    void sendPacket(MsgType type, const uint8_t *payload = nullptr, uint8_t payloadLength = 0)
    {
        char data[payloadLength + 4]{0};
        data[0] = REQUEST_HEAD;
        data[1] = static_cast<char>(type);
        data[2] = payloadLength;
        if (payload)
            memcpy(data + 3, payload, payloadLength);

        data[payloadLength + 3] = checksum(payload, payloadLength);
        Serial3.write(data, payloadLength + 4);
    }

    bool getResponse(Response *resp)
    {
        auto avail = Serial3.available();
        if (!avail) // No data received
            return false;

        if (!(Serial3.read() == RESPONSE_HEAD)) // Invalid head
            return false;

        resp->ack = Serial3.read();
        resp->payloadLength = Serial3.read();
        if (Serial3.readBytes(resp->payload, resp->payloadLength) != resp->payloadLength) // Payload loss
            return false;

        if (!resp->payloadLength) // No payoload, directly fuck off
            return true;

        if (checksum(resp->payload, resp->payloadLength) != Serial3.read()) // Checksum error
            return false;

        return true;
    }

    StatusCode begin()
    {
        Serial3.begin(115200);
        sendPacket(PROBE);
        Response resp;
        if(getResponse(&resp))
            if (resp.ack)
                return OK;

        return FUCK;
    }

    StatusCode setSpeedRPM(uint16_t rpm){

        return OK;
    }
} // namespace RoverIO
