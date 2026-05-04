#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>

#include <cstdint>
#include <vector>

namespace Net
{
    struct PendingMessage
    {
        std::vector<char> data;
        uint32_t seq = 0;
        float timer = 0.0f;
        int retries = 0;
    };

    struct Peer
    {
        int peerId = 0;

        sockaddr_storage addr{};
        int addrLen = 0;

        uint32_t nextSeq = 1;
        uint32_t lastReceivedSeq = 0;

        std::vector<PendingMessage> resendQueue;
    };

#pragma pack(push, 1)
    struct GlobalCommandPayload
    {
        uint8_t commandType = 0;

        // Used by SceneChange
        int32_t sceneIndex = -1;

        // Used by GravityOnOff
        uint8_t gravityEnabled = 1;

        // padding (explicit so size is predictable)
        uint8_t _pad0 = 0;
        uint16_t _pad1 = 0;
    };
#pragma pack(pop)

    enum class GlobalCommandType : uint8_t
    {
        SceneChange = 1,
        GravityOnOff = 2
    };
}