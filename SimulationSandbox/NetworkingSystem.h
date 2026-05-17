#pragma once
#include "UdpSocket.h"
#include "NetProtocol.h"
#include "NetPeer.h"
#include "PeerConfig.h"
#include <unordered_set>
#include <deque>
#include <vector>
#include <deque>
#include <atomic>
#include <random>

namespace Net
{
    struct NetworkStats
    {
        uint32_t controlPacketsReceived = 0;
        uint32_t snapshotPacketsSent = 0;
        uint32_t snapshotPacketsSendFailed = 0;
        uint32_t snapshotPacketsReceived = 0;
        uint32_t snapshotPacketsDropped = 0;
        uint32_t snapshotPacketsDelayed = 0;
        uint32_t snapshotPacketsSkippedInactivePeer = 0;
        uint32_t reliableResends = 0;

        uint32_t globalCommandsSent = 0;
        uint32_t globalCommandsReceived = 0;
        uint32_t spawnPacketsSent = 0;
        uint32_t spawnPacketsReceived = 0;

        uint32_t delayedOutgoingSnapshotPackets = 0;
        uint32_t delayedIncomingSnapshotPackets = 0;
        uint32_t discoveryPacketsSent = 0;
        uint32_t discoveryPacketsReceived = 0;
        uint32_t peersDiscovered = 0;
        uint32_t endpointsLearnedFromControl = 0;
    };

    struct SnapshotImpairmentSettings
    {
        bool enabled = false;
        float latencyMs = 0.0f;
        float jitterMs = 0.0f;
        float dropPercent = 0.0f;
    };

    struct PeerDebugInfo
    {
        int peerId = 0;
        bool active = false;
        bool helloReceived = false;
        bool welcomeReceived = false;
        double lastRttMs = -1.0;
        double avgRttMs = -1.0;
        double jitterMs = 0.0;
        uint32_t pingsSent = 0;
        uint32_t pongsReceived = 0;
        uint32_t pingsTimedOut = 0;
        uint32_t pendingPings = 0;
    };

    class NetworkingSystem
    {
    public:
        bool Init(const PeerConfig& cfg);
        void Shutdown();
        NetworkStats GetStats() const;
        std::vector<int> GetActivePeerIds() const;
        std::vector<PeerDebugInfo> GetPeerDebugInfo() const;
        void Update(float dt);
        void UpdateReceive(float dt);
        void UpdateSend(float dt);

        // ------------------------------------------------------------
        // Global commands (reliable)
        // ------------------------------------------------------------
        void SendSceneChange(int sceneIndex, uint32_t sceneGeneration);
        void SendGravityEnabled(bool enabled);

        bool PopReceivedGlobalCommand(GlobalCommandPayload& out);

        // ------------------------------------------------------------
        // State snapshots (unreliable)
        // ------------------------------------------------------------
        void SendStateSnapshot(
            uint32_t tick,
            uint32_t sceneGeneration,
            const StateSnapshotItem* items,
            uint32_t count);
        // Pops the latest received snapshot (single-slot mailbox).
        // Returns true if one was available.
        bool PopReceivedStateSnapshot(std::vector<StateSnapshotItem>& outItems, uint32_t& outTick);

        // ------------------------------------------------------------
        // Spawn events (reliable)
        // ------------------------------------------------------------
        void SendSpawnObject(const SpawnObjectPayload& payload);
        bool PopReceivedSpawnObject(SpawnObjectPayload& out);

        void SetSnapshotImpairment(const SnapshotImpairmentSettings& settings);

        void ClearSnapshotBacklog();
        void PauseSnapshotTraffic(float seconds);
        void ClearSceneObjectTraffic();
        void SetCurrentSceneGeneration(uint32_t generation);
        uint32_t m_currentSceneGeneration = 1;

    private:
        enum class PacketChannel
        {
            Control,
            Snapshot
        };

        void SendHello();
        void HandlePacket(const sockaddr_storage& from, const char* data, int size, PacketChannel channel);
        void ReceiveControlPacketsFully();
        void UpdateReliableResendsOnControlSocket(float dt);
        void ReceiveSnapshotPacketsWithBudget();
        void DeliverDelayedOutgoingSnapshotsWithBudget(float dt);
        void DeliverDelayedIncomingSnapshotsIfStillUsed(float dt);
        void SendHelloPackets(float dt);
        void SendPingPackets(float dt);
        void SendDiscoveryPackets(float dt);

        void SendGlobalCommand(const GlobalCommandPayload& payload);
        bool ShouldDropSnapshotPacket() const;
        float SampleSnapshotDelaySeconds() const;
        NetworkStats m_stats{};

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

        UdpSocket m_controlSocket;
        UdpSocket m_snapshotSocket;

        int m_localPeerId = 0;
        std::vector<Peer> m_peers;

        char m_recvBuffer[1500];

        // ---- GLOBAL_COMMAND mailbox ----
        std::atomic<bool> m_hasPendingGlobal{ false };
        GlobalCommandPayload m_pendingGlobal{};

        struct ReceivedSnapshotChunk
        {
            uint32_t tick = 0;
            std::vector<StateSnapshotItem> items;
        };

        std::deque<ReceivedSnapshotChunk> m_pendingSnapshotChunks;

        std::deque<SpawnObjectPayload> m_pendingSpawnObjects;

        SnapshotImpairmentSettings m_snapshotImpairment{};
        mutable std::mt19937 m_rng{ std::random_device{}() };
        mutable std::uniform_real_distribution<float> m_unitDist{ 0.0f, 1.0f };
        std::vector<DelayedOutgoingSnapshot> m_delayedOutgoingSnapshots;
        std::vector<DelayedIncomingSnapshot> m_delayedIncomingSnapshots;
        float m_snapshotPauseSeconds = 0.0f;
        uint16_t m_controlBindPort = 0;
        uint16_t m_snapshotBindPort = 0;
        float m_discoveryTimerSec = 0.0f;

        // Used to send large snapshots over several network ticks instead of all at once.
        uint32_t m_snapshotSendCursor = 0;

        // Keep this small enough that control messages are never starved.
        uint32_t m_maxSnapshotPacketsPerSend = 8;
    };
}
