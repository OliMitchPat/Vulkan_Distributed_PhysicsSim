#define NOMINMAX
#include "NetworkingSystem.h"
#include "NetAddress.h"

#include <iostream>
#include <cstring>
#include <algorithm>
#include <cmath>


namespace Net
{
    // ============================================================
    // Endpoint comparison + peer lookup helpers
    // ============================================================

    static bool SockAddrEqualEndpoint(const sockaddr_storage& a, const sockaddr_storage& b)
    {
        if (a.ss_family != b.ss_family) return false;

        if (a.ss_family == AF_INET)
        {
            const sockaddr_in* aa = reinterpret_cast<const sockaddr_in*>(&a);
            const sockaddr_in* bb = reinterpret_cast<const sockaddr_in*>(&b);
            return (aa->sin_port == bb->sin_port) &&
                (aa->sin_addr.s_addr == bb->sin_addr.s_addr);
        }

        if (a.ss_family == AF_INET6)
        {
            const sockaddr_in6* aa = reinterpret_cast<const sockaddr_in6*>(&a);
            const sockaddr_in6* bb = reinterpret_cast<const sockaddr_in6*>(&b);
            return (aa->sin6_port == bb->sin6_port) &&
                (std::memcmp(&aa->sin6_addr, &bb->sin6_addr, sizeof(in6_addr)) == 0);
        }

        return false;
    }

    static Peer* FindPeerByAddr(std::vector<Peer>& peers, const sockaddr_storage& from)
    {
        for (auto& p : peers)
        {
            if (p.addrLen == 0) continue;
            if (SockAddrEqualEndpoint(from, p.addr))
                return &p;
        }
        return nullptr;
    }

    static Peer* FindPeerById(std::vector<Peer>& peers, int peerId)
    {
        for (auto& p : peers)
            if (p.peerId == peerId)
                return &p;
        return nullptr;
    }

    // ============================================================
    // ACK sender
    // ============================================================
    static void SendAck(UdpSocket& socket, Peer& peer, int localPeerId, uint32_t seq)
    {
        MsgHeader ack{};
        ack.msgType = (uint8_t)MsgType::ACK;
        ack.peerId = (uint8_t)localPeerId;
        ack.ack = seq;

        socket.Send(peer.addr, peer.addrLen, &ack, (int)sizeof(ack));
    }

    // ============================================================
    // Reliable send helper (enqueue for retransmit)
    // ============================================================
    static void SendReliable(UdpSocket& socket, Peer& peer, const void* data, int size)
    {
        socket.Send(peer.addr, peer.addrLen, data, size);

        PendingMessage msg;
        msg.data.assign(reinterpret_cast<const char*>(data),
            reinterpret_cast<const char*>(data) + size);
        msg.seq = reinterpret_cast<const MsgHeader*>(data)->seq;
        msg.timer = 0.0f;
        msg.retries = 0;

        peer.resendQueue.push_back(std::move(msg));
    }

    // ============================================================
    // Init / Shutdown
    // ============================================================
    bool NetworkingSystem::Init(const PeerConfig& cfg)
    {
        std::string err;
        if (!m_socket.Open(cfg.bind_ip, cfg.bind_port, err))
        {
            std::cout << "Socket error: " << err << "\n";
            return false;
        }

        m_localPeerId = cfg.peer_id;

        m_peers.clear();
        for (const auto& p : cfg.RemotePeers())
        {
            if (p.peerId == cfg.peer_id)
                continue;

            sockaddr_storage addr{};
            std::string err2;

            if (!ResolveAddress(p.host, p.port, addr, err2))
            {
                std::cout << "Resolve failed: " << err2 << "\n";
                continue;
            }

            Peer peer;
            peer.peerId = p.peerId;
            peer.addr = addr;
            peer.addrLen = Net::SockaddrLen(addr);
            m_peers.push_back(peer);
        }

        SendHello();
        return true;
    }

    void NetworkingSystem::Shutdown()
    {
        m_socket.Close();
    }

    // ============================================================
    // Send HELLO (unreliable)
    // ============================================================
    void NetworkingSystem::SendHello()
    {
        MsgHeader msg{};
        msg.msgType = (uint8_t)MsgType::HELLO;
        msg.peerId = (uint8_t)m_localPeerId;

        for (auto& p : m_peers)
            m_socket.Send(p.addr, p.addrLen, &msg, (int)sizeof(msg));
    }

    // ============================================================
    // Update
    // ============================================================
    void NetworkingSystem::Update(float dt)
    {
        // ---- Deliver delayed outgoing snapshot packets ----
        for (size_t i = 0; i < m_delayedOutgoingSnapshots.size();)
        {
            auto& p = m_delayedOutgoingSnapshots[i];
            p.delaySec -= dt;
            if (p.delaySec <= 0.0f)
            {
                m_socket.Send(p.addr, p.addrLen, p.payload.data(), (int)p.payload.size());
                m_delayedOutgoingSnapshots.erase(m_delayedOutgoingSnapshots.begin() + (int)i);
                continue;
            }
            ++i;
        }

        sockaddr_storage from{};

        // ---- Receive loop ----
        int size = 0;
        while ((size = m_socket.Receive(from, m_recvBuffer, (int)sizeof(m_recvBuffer))) > 0)
        {
            // Optional: lightweight debug for GLOBAL_COMMAND only
            if (size >= (int)sizeof(MsgHeader))
            {
                const MsgHeader* h = reinterpret_cast<const MsgHeader*>(m_recvBuffer);
                if (h->protocolVersion == PROTOCOL_VERSION &&
                    (MsgType)h->msgType == MsgType::GLOBAL_COMMAND)
                {
                    std::cout
                        << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GOT GLOBAL_COMMAND"
                        << " size=" << size
                        << " fromPeerId=" << (int)h->peerId
                        << " seq=" << h->seq
                        << "\n";
                }
            }

            HandlePacket(from, m_recvBuffer, size);
        }

        // ---- Deliver delayed incoming snapshots to mailbox ----
        for (size_t i = 0; i < m_delayedIncomingSnapshots.size();)
        {
            auto& p = m_delayedIncomingSnapshots[i];
            p.delaySec -= dt;
            if (p.delaySec <= 0.0f)
            {
                m_pendingSnapshotTick = p.tick;
                m_pendingSnapshotItems = p.items;
                m_hasPendingSnapshot.store(true, std::memory_order_release);
                m_delayedIncomingSnapshots.erase(m_delayedIncomingSnapshots.begin() + (int)i);
                continue;
            }
            ++i;
        }

        // ---- Resend logic (reliable queue) ----
        constexpr float RESEND_INTERVAL_SEC = 0.05f;
        constexpr int   MAX_RETRIES = 10;

        for (auto& peer : m_peers)
        {
            for (size_t i = 0; i < peer.resendQueue.size(); /* increment inside */)
            {
                auto& msg = peer.resendQueue[i];
                msg.timer += dt;

                if (msg.timer >= RESEND_INTERVAL_SEC)
                {
                    if (msg.retries >= MAX_RETRIES)
                    {
                        std::cout << "[NET] Drop reliable msg seq=" << msg.seq
                            << " to peer " << peer.peerId
                            << " (max retries reached)\n";

                        peer.resendQueue.erase(peer.resendQueue.begin() + i);
                        continue;
                    }

                    m_socket.Send(peer.addr, peer.addrLen,
                        msg.data.data(), (int)msg.data.size());

                    msg.timer = 0.0f;
                    msg.retries++;
                }

                ++i;
            }
        }
    }

    // ============================================================
    // Handle incoming packet
    // ============================================================
    void NetworkingSystem::HandlePacket(const sockaddr_storage& from, const char* data, int size)
    {
        if (size < (int)sizeof(MsgHeader))
            return;

        const MsgHeader* hdr = reinterpret_cast<const MsgHeader*>(data);

        if (hdr->protocolVersion != PROTOCOL_VERSION)
            return;

        if (hdr->peerId == 0)
            return;

        Peer* peer = FindPeerByAddr(m_peers, from);
        if (!peer)
        {
            peer = FindPeerById(m_peers, (int)hdr->peerId);
            if (!peer) return;

            // Learn sender endpoint (so future ACKs/resends go to the right place)
            peer->addr = from;
            peer->addrLen = Net::SockaddrLen(from);
        }
        // Safety: ensure the endpoint-mapped peer id matches the header.
        // Prevents accidentally clearing a wrong peer's resend queue.

        switch ((MsgType)hdr->msgType)
        {
        case MsgType::HELLO:
        {
            std::cout << "Received HELLO from peer " << (int)hdr->peerId << "\n";

            MsgHeader reply{};
            reply.msgType = (uint8_t)MsgType::WELCOME;
            reply.peerId = (uint8_t)m_localPeerId;
            reply.seq = peer->nextSeq++;

            m_socket.Send(peer->addr, peer->addrLen, &reply, (int)sizeof(reply));
            break;
        }

        case MsgType::WELCOME:
        {
            std::cout << "Received WELCOME from peer " << (int)hdr->peerId << "\n";
            break;
        }

        case MsgType::ACK:
        {
            const uint32_t ackSeq = hdr->ack;

            auto& queue = peer->resendQueue;
            const size_t before = queue.size();

            queue.erase(
                std::remove_if(queue.begin(), queue.end(),
                    [&](const PendingMessage& m) { return m.seq == ackSeq; }),
                queue.end());

            const size_t after = queue.size();

            std::cout << "[ACK] recv from hdrPeerId=" << (int)hdr->peerId
                << " mappedPeer=" << peer->peerId
                << " ackSeq=" << ackSeq
                << " q " << before << " -> " << after << "\n";
            break;
        }
        case MsgType::GLOBAL_COMMAND:
        {
            // Always ACK reliable messages (even duplicates)
            SendAck(m_socket, *peer, m_localPeerId, hdr->seq);

            std::cout << "///////////////////////////////////////////////////////////////////// RAW GLOBAL from peer=" << (int)hdr->peerId
                << " seq=" << hdr->seq
                << " last=" << peer->lastReceivedSeq
                << " size=" << size << "\n";

            // Duplicate suppression
            if (hdr->seq != 0 && hdr->seq <= peer->lastReceivedSeq)
                break;

            peer->lastReceivedSeq = hdr->seq;

            if (size < (int)sizeof(MsgHeader) + (int)sizeof(GlobalCommandPayload))
                break;

            const GlobalCommandPayload* payload =
                reinterpret_cast<const GlobalCommandPayload*>(data + sizeof(MsgHeader));

            // Store latest for PopReceivedGlobalCommand()
            m_pendingGlobal = *payload;
            m_hasPendingGlobal.store(true, std::memory_order_release);
            break;
        }

        case MsgType::STATE_SNAPSHOT:
        {
            if (ShouldDropSnapshotPacket())
                break;

            if (size < (int)(sizeof(MsgHeader) + sizeof(StateSnapshotHeader)))
                break;

            const StateSnapshotHeader* sh =
                reinterpret_cast<const StateSnapshotHeader*>(data + sizeof(MsgHeader));

            const int count = (int)sh->count;
            const int expectedSize =
                (int)(sizeof(MsgHeader) + sizeof(StateSnapshotHeader) + count * sizeof(StateSnapshotItem));

            if (count < 0 || expectedSize > size)
                break;

            const StateSnapshotItem* items =
                reinterpret_cast<const StateSnapshotItem*>(data + sizeof(MsgHeader) + sizeof(StateSnapshotHeader));

            const float delaySec = SampleSnapshotDelaySeconds();
            if (delaySec > 0.0f)
            {
                DelayedIncomingSnapshot delayed{};
                delayed.tick = hdr->tick;
                delayed.items.assign(items, items + count);
                delayed.delaySec = delaySec;
                m_delayedIncomingSnapshots.push_back(std::move(delayed));
            }
            else
            {
                m_pendingSnapshotTick = hdr->tick;
                m_pendingSnapshotItems.assign(items, items + count);
                m_hasPendingSnapshot.store(true, std::memory_order_release);
            }
            break;
        }

        case MsgType::SPAWN_OBJECT:
        {
            // Reliable channel
            SendAck(m_socket, *peer, m_localPeerId, hdr->seq);

            // Duplicate suppression
            if (hdr->seq != 0 && hdr->seq <= peer->lastReceivedSeq)
                break;

            peer->lastReceivedSeq = hdr->seq;

            if (size < (int)sizeof(MsgHeader) + (int)sizeof(SpawnObjectPayload))
                break;

            const SpawnObjectPayload* payload =
                reinterpret_cast<const SpawnObjectPayload*>(data + sizeof(MsgHeader));

            m_pendingSpawnObjects.push_back(*payload);
            break;
        }

        default:
            break;
        }
    }

    // ============================================================
    // GLOBAL_COMMAND sending (reliable)
    // ============================================================
    void NetworkingSystem::SendSceneChange(int sceneIndex)
    {
        GlobalCommandPayload p{};
        p.commandType = (uint8_t)GlobalCommandType::SceneChange;
        p.sceneIndex = sceneIndex;
        SendGlobalCommand(p);
    }

    void NetworkingSystem::SendGravityEnabled(bool enabled)
    {
        GlobalCommandPayload p{};
        p.commandType = (uint8_t)GlobalCommandType::GravityOnOff;
        p.gravityEnabled = enabled ? 1 : 0;
        SendGlobalCommand(p);
    }

    bool NetworkingSystem::PopReceivedGlobalCommand(GlobalCommandPayload& out)
    {
        if (!m_hasPendingGlobal.exchange(false, std::memory_order_acq_rel))
            return false;

        out = m_pendingGlobal;
        return true;
    }

    void NetworkingSystem::SendGlobalCommand(const GlobalCommandPayload& payload)
    {
        for (auto& peer : m_peers)
        {
            MsgHeader hdr{};
            hdr.msgType = (uint8_t)MsgType::GLOBAL_COMMAND;
            hdr.peerId = (uint8_t)m_localPeerId;
            hdr.seq = peer.nextSeq++;

            char buffer[256];
            std::memcpy(buffer, &hdr, sizeof(hdr));
            std::memcpy(buffer + sizeof(hdr), &payload, sizeof(payload));

            SendReliable(m_socket, peer, buffer, (int)(sizeof(hdr) + sizeof(payload)));
        }
    }

    void NetworkingSystem::SendSpawnObject(const SpawnObjectPayload& payload)
    {
        for (auto& peer : m_peers)
        {
            MsgHeader hdr{};
            hdr.msgType = (uint8_t)MsgType::SPAWN_OBJECT;
            hdr.peerId = (uint8_t)m_localPeerId;
            hdr.seq = peer.nextSeq++;

            char buffer[256];
            std::memcpy(buffer, &hdr, sizeof(hdr));
            std::memcpy(buffer + sizeof(hdr), &payload, sizeof(payload));

            SendReliable(m_socket, peer, buffer, (int)(sizeof(hdr) + sizeof(payload)));
        }
    }

    bool NetworkingSystem::PopReceivedSpawnObject(SpawnObjectPayload& out)
    {
        if (m_pendingSpawnObjects.empty())
            return false;

        out = m_pendingSpawnObjects.front();
        m_pendingSpawnObjects.pop_front();
        return true;
    }

    // ============================================================
    // STATE_SNAPSHOT sending (unreliable)
    // ============================================================
    void NetworkingSystem::SendStateSnapshot(uint32_t tick, const StateSnapshotItem* items, uint16_t count)
    {
        if (!items || count == 0)
            return;

        const int bytes =
            (int)sizeof(MsgHeader) +
            (int)sizeof(StateSnapshotHeader) +
            (int)count * (int)sizeof(StateSnapshotItem);

        if (bytes > (int)sizeof(m_recvBuffer))
        {
            std::cout << "[NET] STATE_SNAPSHOT too large: " << bytes << " bytes (count=" << count << ")\n";
            return;
        }

        char buffer[1500];

        MsgHeader hdr{};
        hdr.msgType = (uint8_t)MsgType::STATE_SNAPSHOT;
        hdr.peerId = (uint8_t)m_localPeerId;
        hdr.tick = tick;

        StateSnapshotHeader sh{};
        sh.count = count;

        std::memcpy(buffer, &hdr, sizeof(hdr));
        std::memcpy(buffer + sizeof(hdr), &sh, sizeof(sh));
        std::memcpy(buffer + sizeof(hdr) + sizeof(sh), items, count * sizeof(StateSnapshotItem));

        for (auto& peer : m_peers)
        {
            if (ShouldDropSnapshotPacket())
                continue;

            const float delaySec = SampleSnapshotDelaySeconds();
            if (delaySec > 0.0f)
            {
                DelayedOutgoingSnapshot delayed{};
                delayed.addr = peer.addr;
                delayed.addrLen = peer.addrLen;
                delayed.payload.assign(buffer, buffer + bytes);
                delayed.delaySec = delaySec;
                m_delayedOutgoingSnapshots.push_back(std::move(delayed));
            }
            else
            {
                m_socket.Send(peer.addr, peer.addrLen, buffer, bytes);
            }
        }
    }

    bool NetworkingSystem::PopReceivedStateSnapshot(std::vector<StateSnapshotItem>& outItems, uint32_t& outTick)
    {
        if (!m_hasPendingSnapshot.exchange(false, std::memory_order_acq_rel))
            return false;

        outTick = m_pendingSnapshotTick;
        outItems = m_pendingSnapshotItems;
        return true;
    }

    void NetworkingSystem::SetSnapshotImpairment(const SnapshotImpairmentSettings& settings)
    {
        m_snapshotImpairment.enabled = settings.enabled;
        m_snapshotImpairment.latencyMs = std::max(0.0f, settings.latencyMs);
        m_snapshotImpairment.jitterMs = std::max(0.0f, settings.jitterMs);
        m_snapshotImpairment.dropPercent = std::max(0.0f, std::min(settings.dropPercent, 100.0f));
    }

    bool NetworkingSystem::ShouldDropSnapshotPacket() const
    {
        if (!m_snapshotImpairment.enabled)
            return false;

        const float drop = m_snapshotImpairment.dropPercent;
        if (drop <= 0.0f)
            return false;

        return (m_unitDist(m_rng) * 100.0f) < drop;
    }

    float NetworkingSystem::SampleSnapshotDelaySeconds() const
    {
        if (!m_snapshotImpairment.enabled)
            return 0.0f;

        const float latencyMs = m_snapshotImpairment.latencyMs;
        const float jitterMs = m_snapshotImpairment.jitterMs;
        if (latencyMs <= 0.0f && jitterMs <= 0.0f)
            return 0.0f;

        float delayMs = latencyMs;
        if (jitterMs > 0.0f)
        {
            const float jitter = (m_unitDist(m_rng) * 2.0f - 1.0f) * jitterMs;
            delayMs += jitter;
        }

        delayMs = std::max(0.0f, delayMs);
        return delayMs * 0.001f;
    }

}
