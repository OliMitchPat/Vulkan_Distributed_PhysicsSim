#pragma once
#include <cstdint>

namespace Net
{
    static constexpr uint32_t PROTOCOL_VERSION = 1;

    enum class MsgType : uint8_t
    {
        HELLO = 1,
        WELCOME,
        GLOBAL_COMMAND,
        STATE_SNAPSHOT,
        ACK
    };

#pragma pack(push, 1)
    struct MsgHeader
    {
        uint32_t protocolVersion = PROTOCOL_VERSION;
        uint8_t  msgType = 0;
        uint8_t  peerId = 0;
        uint16_t reserved = 0;

        uint32_t seq = 0;
        uint32_t ack = 0; // piggyback ACK (very good ??)

        uint32_t tick = 0; // for snapshots
    };
#pragma pack(pop)
}