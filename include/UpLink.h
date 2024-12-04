#pragma once

#include <Arduino.h>

namespace RoverIO
{
    
    enum StatusCode
    {
        OK = 0,
        FUCK,
        // To be continued
    };

    enum MsgType
    {
        PROBE = 0,
        SET_SPEED,
        GET_SPEED,
        SET_DIR,
        GET_DIR,
        // To be continueds
    };

    enum IfAck
    {
        NACK = 0,
        ACK
    };

    struct Response
    {
        bool ack = false;
        uint8_t payloadLength;
        uint8_t payload[32];
    };

    struct Request
    {
        MsgType type;
        uint8_t payloadLength;
        uint8_t payload[32];
    };
    

    StatusCode begin();

    StatusCode setSpeedRPM(uint16_t rpm);

    StatusCode getSpeedRPM(uint16_t *rpm);

    StatusCode setDirection(bool reverse);

    StatusCode getDirection(bool *reverse);

} // namespace RoverIO
