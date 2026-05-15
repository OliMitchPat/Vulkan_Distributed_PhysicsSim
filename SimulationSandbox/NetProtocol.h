#pragma once
#include <cstdint>

namespace Net
{
    static constexpr uint32_t PROTOCOL_VERSION = 3;

    enum class MsgType : uint8_t
    {
        HELLO = 1,
        WELCOME,
        GLOBAL_COMMAND,
        STATE_SNAPSHOT,
        ACK,
        SPAWN_OBJECT
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

        uint16_t chunkIndex = 0;

        uint16_t chunkCount = 1;

        uint16_t reserved = 0;

        uint32_t sceneGeneration = 0;
    };

    struct StateSnapshotItem
    {
        uint32_t objectId = 0;  // Entity id (deterministic)
        NetVec3  pos;
        NetQuat  rot;
        NetVec3  linVel;
        NetVec3  angVel;
    };

    enum class SpawnShapeType : uint8_t
    {
        Sphere = 1,
        Cylinder = 2,
        Capsule = 3,
        Cuboid = 4
    };

    struct SpawnObjectPayload
    {
        uint32_t objectId = 0;
        uint32_t sceneGeneration = 0;
        uint8_t ownerId = 0;      // 0..3 maps to ONE..FOUR
        uint8_t shapeType = 0;    // SpawnShapeType
        uint16_t reserved = 0;

        NetVec3 pos;
        NetQuat rot;
        NetVec3 linVel;
        NetVec3 angVel;

        // shape parameters:
        // Sphere:  radius
        // Cylinder/Capsule: radius + height
        // Cuboid: sizeX,sizeY,sizeZ
        float radius = 0.5f;
        float height = 1.0f;
        float sizeX = 1.0f;
        float sizeY = 1.0f;
        float sizeZ = 1.0f;

        char material[32]{};
    };
#pragma pack(pop)
}
