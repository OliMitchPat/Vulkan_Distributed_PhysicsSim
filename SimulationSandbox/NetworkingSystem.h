#pragma once
#include "UdpSocket.h"
#include "NetProtocol.h"
#include "NetPeer.h"
#include "PeerConfig.h"

#include <vector>
#include <deque>
#include <atomic>
#include <random>

namespace Net
{
    struct SnapshotImpairmentSettings
    {
        bool enabled = false;
        float latencyMs = 0.0f;
        float jitterMs = 0.0f;
        float dropPercent = 0.0f;
    };

    class NetworkingSystem
    {
    public:
        bool Init(const PeerConfig& cfg);
        void Shutdown();

        void Update(float dt);

        // ------------------------------------------------------------
        // Global commands (reliable)
        // ------------------------------------------------------------
        void SendSceneChange(int sceneIndex);
        void SendGravityEnabled(bool enabled);

        bool PopReceivedGlobalCommand(GlobalCommandPayload& out);

        // ------------------------------------------------------------
        // State snapshots (unreliable)
        // ------------------------------------------------------------
        void SendStateSnapshot(uint32_t tick, const StateSnapshotItem* items, uint16_t count);

        // Pops the latest received snapshot (single-slot mailbox).
        // Returns true if one was available.
        bool PopReceivedStateSnapshot(std::vector<StateSnapshotItem>& outItems, uint32_t& outTick);

        // ------------------------------------------------------------
        // Spawn events (reliable)
        // ------------------------------------------------------------
        void SendSpawnObject(const SpawnObjectPayload& payload);
        bool PopReceivedSpawnObject(SpawnObjectPayload& out);

        void SetSnapshotImpairment(const SnapshotImpairmentSettings& settings);

    private:
        void SendHello();
        void HandlePacket(const sockaddr_storage& from, const char* data, int size);

        void SendGlobalCommand(const GlobalCommandPayload& payload);
        bool ShouldDropSnapshotPacket() const;
        float SampleSnapshotDelaySeconds() const;

    private:
        struct DelayedOutgoingSnapshot
        {
            sockaddr_storage addr{};
            int addrLen = 0;
            std::vector<char> payload;
            float delaySec = 0.0f;
        };

        struct DelayedIncomingSnapshot
        {
            uint32_t tick = 0;
            std::vector<StateSnapshotItem> items;
            float delaySec = 0.0f;
        };

        UdpSocket m_socket;

        int m_localPeerId = 0;
        std::vector<Peer> m_peers;

        char m_recvBuffer[1500];

        // ---- GLOBAL_COMMAND mailbox ----
        std::atomic<bool> m_hasPendingGlobal{ false };
        GlobalCommandPayload m_pendingGlobal{};

        // ---- STATE_SNAPSHOT mailbox (latest only) ----
        std::atomic<bool> m_hasPendingSnapshot{ false };
        uint32_t m_pendingSnapshotTick = 0;
        std::vector<StateSnapshotItem> m_pendingSnapshotItems;

        std::deque<SpawnObjectPayload> m_pendingSpawnObjects;

        SnapshotImpairmentSettings m_snapshotImpairment{};
        mutable std::mt19937 m_rng{ std::random_device{}() };
        mutable std::uniform_real_distribution<float> m_unitDist{ 0.0f, 1.0f };
        std::vector<DelayedOutgoingSnapshot> m_delayedOutgoingSnapshots;
        std::vector<DelayedIncomingSnapshot> m_delayedIncomingSnapshots;
    };
}
