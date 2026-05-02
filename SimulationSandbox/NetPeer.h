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

    struct GlobalCommandPayload
    {
        uint8_t commandType = 0;
    };

    enum class GlobalCommandType : uint8_t
    {
        ToggleGravity = 1,
        ResetScene = 2
    };
}