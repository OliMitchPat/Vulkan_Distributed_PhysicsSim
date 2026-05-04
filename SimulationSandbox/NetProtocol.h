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
        uint32_t ack = 0;

        uint32_t tick = 0; // for snapshots
    };

    // ============================================================
    // STATE_SNAPSHOT (Milestone 5)
    //
    // Packet layout:
    //   MsgHeader hdr
    //   StateSnapshotHeader sh
    //   StateSnapshotItem items[sh.count]
    // ============================================================

    struct NetVec3
    {
        float x = 0, y = 0, z = 0;
    };

    struct NetQuat
    {
        float x = 0, y = 0, z = 0, w = 1;
    };

    struct StateSnapshotHeader
    {
        uint16_t count = 0;
        uint16_t reserved = 0;
    };

    struct StateSnapshotItem
    {
        uint32_t objectId = 0;  // Entity id (deterministic)
        NetVec3  pos;
        NetQuat  rot;
        NetVec3  linVel;
        NetVec3  angVel;
    };
#pragma pack(pop)
}