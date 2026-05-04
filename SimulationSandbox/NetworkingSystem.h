#pragma once
#include "UdpSocket.h"
#include "NetProtocol.h"
#include "NetPeer.h"
#include "PeerConfig.h"

#include <vector>
#include <atomic>

namespace Net
{
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

    private:
        void SendHello();
        void HandlePacket(const sockaddr_storage& from, const char* data, int size);

        void SendGlobalCommand(const GlobalCommandPayload& payload);

    private:
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
    };
}