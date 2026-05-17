#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <unordered_set>
#include <deque>
#include <cstdint>
#include <vector>
#include <unordered_map>

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

        sockaddr_storage controlAddr{};
        int controlAddrLen = 0;

        sockaddr_storage snapshotAddr{};
        int snapshotAddrLen = 0;

        uint32_t nextSeq = 1;
        bool active = false;
        std::unordered_set<uint32_t> receivedReliableSeqs;
        std::deque<uint32_t> receivedReliableSeqOrder;
        std::vector<PendingMessage> resendQueue;

        float pingTimerSec = 0.0f;
        uint32_t nextPingId = 1;
        uint32_t pingsSent = 0;
        uint32_t pongsReceived = 0;
        uint32_t pingsTimedOut = 0;
        double lastRttMs = -1.0;
        double avgRttMs = -1.0;
        double jitterMs = 0.0;
        std::unordered_map<uint32_t, double> pendingPings;
    };

#pragma pack(push, 1)
    struct GlobalCommandPayload
    {
        uint8_t commandType = 0;
        uint8_t gravityEnabled = 0;
        uint16_t reserved = 0;

        int32_t sceneIndex = -1;
        uint32_t sceneGeneration = 0;
    };
#pragma pack(pop)

    enum class GlobalCommandType : uint8_t
    {
        SceneChange = 1,
        GravityOnOff = 2
    };
}
