#define NOMINMAX
#include "NetworkingSystem.h"
#include "NetAddress.h"

#include <iostream>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace Net
{
    constexpr int MAX_CONTROL_PACKETS_PER_UPDATE = 256;
    constexpr int MAX_SNAPSHOT_PACKETS_PER_UPDATE = 64;
    constexpr int MAX_DELAYED_SNAPSHOT_SENDS_PER_UPDATE = 32;
    constexpr size_t MAX_DELAYED_OUTGOING_SNAPSHOTS = 256;

    static double NowSeconds()
    {
        using clock = std::chrono::steady_clock;
        return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
    }

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

    static Peer* FindPeerByControlAddr(std::vector<Peer>& peers, const sockaddr_storage& from)
    {
        for (auto& p : peers)
        {
            if (p.controlAddrLen == 0) continue;
            if (SockAddrEqualEndpoint(from, p.controlAddr))
                return &p;
        }
        return nullptr;
    }

    static Peer* FindPeerBySnapshotAddr(std::vector<Peer>& peers, const sockaddr_storage& from)
    {
        for (auto& p : peers)
        {
            if (p.snapshotAddrLen == 0) continue;
            if (SockAddrEqualEndpoint(from, p.snapshotAddr))
                return &p;
        }
        return nullptr;
    }

    static Peer* FindPeerById(std::vector<Peer>& peers, int peerId)
    {
        for (auto& p : peers)
        {
            if (p.peerId == peerId)
                return &p;
        }

        return nullptr;
    }

    static void SetSockaddrPort(sockaddr_storage& addr, uint16_t port)
    {
        if (addr.ss_family == AF_INET)
        {
            auto* in = reinterpret_cast<sockaddr_in*>(&addr);
            in->sin_port = htons(port);
        }
        else if (addr.ss_family == AF_INET6)
        {
            auto* in = reinterpret_cast<sockaddr_in6*>(&addr);
            in->sin6_port = htons(port);
        }
    }

    static sockaddr_storage AddressWithPort(sockaddr_storage addr, uint16_t port)
    {
        SetSockaddrPort(addr, port);
        return addr;
    }

    static bool PendingMessageIsType(const PendingMessage& msg, MsgType type)
    {
        if (msg.data.size() < sizeof(MsgHeader))
            return false;

        const MsgHeader* hdr =
            reinterpret_cast<const MsgHeader*>(msg.data.data());

        return (MsgType)hdr->msgType == type;
    }

    // ============================================================
    // Reliable duplicate suppression
    //
    // Do NOT use lastReceivedSeq for duplicate suppression.
    // UDP packets can arrive out of order, especially under latency/jitter.
    // Track received reliable sequence numbers individually instead.
    // ============================================================

    static bool IsDuplicateReliable(Peer& peer, uint32_t seq)
    {
        if (seq == 0)
            return false;

        return peer.receivedReliableSeqs.find(seq) != peer.receivedReliableSeqs.end();
    }

    static void MarkReliableReceived(Peer& peer, uint32_t seq)
    {
        if (seq == 0)
            return;

        constexpr size_t MAX_TRACKED_RELIABLE_SEQS = 1024;

        peer.receivedReliableSeqs.insert(seq);
        peer.receivedReliableSeqOrder.push_back(seq);

        while (peer.receivedReliableSeqOrder.size() > MAX_TRACKED_RELIABLE_SEQS)
        {
            const uint32_t oldSeq = peer.receivedReliableSeqOrder.front();
            peer.receivedReliableSeqOrder.pop_front();
            peer.receivedReliableSeqs.erase(oldSeq);
        }
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

        socket.Send(peer.controlAddr, peer.controlAddrLen, &ack, (int)sizeof(ack));
    }

    static void SendHelloToPeer(UdpSocket& socket, Peer& peer, int localPeerId)
    {
        MsgHeader msg{};
        msg.msgType = (uint8_t)MsgType::HELLO;
        msg.peerId = (uint8_t)localPeerId;

        socket.Send(peer.controlAddr, peer.controlAddrLen, &msg, (int)sizeof(msg));
    }

    // ============================================================
    // Reliable send helper
    //
    // Reliable messages are sent immediately once, then stored for resend.
    // High-priority reliable messages, such as scene changes, are placed
    // at the front of the resend queue.
    // ============================================================

    static void SendReliable(
        UdpSocket& socket,
        Peer& peer,
        const void* data,
        int size,
        bool highPriority = false)
    {
        // Always send immediately once.
        socket.Send(peer.controlAddr, peer.controlAddrLen, data, size);

        PendingMessage msg;
        msg.data.assign(
            reinterpret_cast<const char*>(data),
            reinterpret_cast<const char*>(data) + size);

        msg.seq = reinterpret_cast<const MsgHeader*>(data)->seq;
        msg.timer = 0.0f;
        msg.retries = 0;

        if (highPriority)
            peer.resendQueue.insert(peer.resendQueue.begin(), std::move(msg));
        else
            peer.resendQueue.push_back(std::move(msg));
    }

    // ============================================================
    // Init / Shutdown
    // ============================================================

    bool NetworkingSystem::Init(const PeerConfig& cfg)
    {
        std::string err;
        if (!m_controlSocket.Open(cfg.bind_ip, cfg.control_bind_port, err))
        {
            std::cout << "Control socket error: " << err << "\n";
            return false;
        }

        err.clear();
        if (!m_snapshotSocket.Open(cfg.bind_ip, cfg.snapshot_bind_port, err))
        {
            std::cout << "Snapshot socket error: " << err << "\n";
            m_controlSocket.Close();
            return false;
        }

        m_localPeerId = cfg.peer_id;
        m_controlBindPort = cfg.control_bind_port;
        m_snapshotBindPort = cfg.snapshot_bind_port;

        m_peers.clear();

        for (const auto& p : cfg.RemotePeers())
        {
            if (p.peerId == cfg.peer_id)
                continue;

            sockaddr_storage controlAddr{};
            sockaddr_storage snapshotAddr{};
            std::string err2;

            if (!ResolveAddress(p.host, p.control_port, controlAddr, err2))
            {
                std::cout << "Resolve control failed: " << err2 << "\n";
                continue;
            }

            if (!ResolveAddress(p.host, p.snapshot_port, snapshotAddr, err2))
            {
                std::cout << "Resolve snapshot failed: " << err2 << "\n";
                continue;
            }

            Peer peer;
            peer.peerId = p.peerId;
            peer.controlAddr = controlAddr;
            peer.controlAddrLen = Net::SockaddrLen(controlAddr);
            peer.snapshotAddr = snapshotAddr;
            peer.snapshotAddrLen = Net::SockaddrLen(snapshotAddr);
            peer.configuredControlPort = p.control_port;
            peer.configuredSnapshotPort = p.snapshot_port;

            m_peers.push_back(std::move(peer));
        }

        SendHello();
        return true;
    }

    void NetworkingSystem::Shutdown()
    {
        m_controlSocket.Close();
        m_snapshotSocket.Close();
    }

    // ============================================================
    // Send HELLO
    // ============================================================

    void NetworkingSystem::SendHello()
    {
        MsgHeader msg{};
        msg.msgType = (uint8_t)MsgType::HELLO;
        msg.peerId = (uint8_t)m_localPeerId;

        for (auto& p : m_peers)
            SendHelloToPeer(m_controlSocket, p, m_localPeerId);
    }

    // ============================================================
    // Update
    //
    // Important ordering:
    // 1. Receive packets first so incoming GLOBAL_COMMAND messages are handled
    //    before this peer spends time flushing snapshot traffic.
    // 2. Resend reliable messages.
    // 3. Deliver delayed outgoing snapshots with a budget.
    //
    // This keeps client 2 -> client 1 scene changes responsive even when
    // client 1 owns many objects and is producing lots of snapshot traffic.
    // ============================================================

    void NetworkingSystem::Update(float dt)
    {
        UpdateReceive(dt);
        UpdateSend(dt);
    }

    void NetworkingSystem::UpdateReceive(float dt)
    {
        ReceiveControlPacketsFully();
        ReceiveSnapshotPacketsWithBudget();
        DeliverDelayedIncomingSnapshotsIfStillUsed(dt);
    }

    void NetworkingSystem::UpdateSend(float dt)
    {
        if (m_snapshotPauseSeconds > 0.0f)
        {
            m_snapshotPauseSeconds = std::max(0.0f, m_snapshotPauseSeconds - dt);
        }

        UpdateReliableResendsOnControlSocket(dt);
        SendDiscoveryPackets(dt);
        SendHelloPackets(dt);
        SendPingPackets(dt);
        DeliverDelayedOutgoingSnapshotsWithBudget(dt);
    }

    void NetworkingSystem::SendHelloPackets(float dt)
    {
        constexpr float HELLO_INTERVAL_SEC = 1.0f;

        for (auto& peer : m_peers)
        {
            if (peer.active)
                continue;

            peer.helloTimerSec += dt;
            if (peer.helloTimerSec < HELLO_INTERVAL_SEC)
                continue;

            peer.helloTimerSec = 0.0f;
            SendHelloToPeer(m_controlSocket, peer, m_localPeerId);
        }
    }

    void NetworkingSystem::SendDiscoveryPackets(float dt)
    {
        constexpr float DISCOVERY_INTERVAL_SEC = 1.0f;

        m_discoveryTimerSec += dt;
        if (m_discoveryTimerSec < DISCOVERY_INTERVAL_SEC)
            return;

        m_discoveryTimerSec = 0.0f;

        struct DiscoveryPacket
        {
            MsgHeader header;
            DiscoveryPayload payload;
        } packet{};

        packet.header.msgType = (uint8_t)MsgType::DISCOVER_SIM_PEER;
        packet.header.peerId = (uint8_t)m_localPeerId;
        packet.payload.peerId = (uint8_t)m_localPeerId;
        packet.payload.controlPort = m_controlBindPort;
        packet.payload.snapshotPort = m_snapshotBindPort;

        for (const auto& peer : m_peers)
        {
            sockaddr_storage broadcastAddr{};
            std::string err;
            if (!ResolveAddress("255.255.255.255", peer.configuredControlPort, broadcastAddr, err))
                continue;

            if (m_controlSocket.Send(broadcastAddr, Net::SockaddrLen(broadcastAddr), &packet, (int)sizeof(packet)))
                ++m_stats.discoveryPacketsSent;
        }
    }

    void NetworkingSystem::SendPingPackets(float dt)
    {
        constexpr float PING_INTERVAL_SEC = 1.0f;
        constexpr double PING_TIMEOUT_SEC = 3.0;
        const double nowSec = NowSeconds();

        for (auto& peer : m_peers)
        {
            for (auto it = peer.pendingPings.begin(); it != peer.pendingPings.end();)
            {
                if ((nowSec - it->second) > PING_TIMEOUT_SEC)
                {
                    it = peer.pendingPings.erase(it);
                    ++peer.pingsTimedOut;
                }
                else
                {
                    ++it;
                }
            }

            peer.pingTimerSec += dt;
            if (peer.pingTimerSec < PING_INTERVAL_SEC)
                continue;

            peer.pingTimerSec = 0.0f;

            struct PingPacket
            {
                MsgHeader header;
                PingPayload payload;
            } packet{};

            packet.header.msgType = (uint8_t)MsgType::PING;
            packet.header.peerId = (uint8_t)m_localPeerId;
            packet.payload.pingId = peer.nextPingId++;
            packet.payload.senderTimeSec = nowSec;

            peer.pendingPings[packet.payload.pingId] = nowSec;
            ++peer.pingsSent;

            m_controlSocket.Send(
                peer.controlAddr,
                peer.controlAddrLen,
                &packet,
                (int)sizeof(packet));
        }
    }

    // ============================================================
    // Handle incoming packet
    // ============================================================

    void NetworkingSystem::ReceiveControlPacketsFully()
    {
        sockaddr_storage from{};
        int size = 0;
        int processed = 0;
        while ((size = m_controlSocket.Receive(from, m_recvBuffer, (int)sizeof(m_recvBuffer))) > 0)
        {
            ++m_stats.controlPacketsReceived;
            HandlePacket(from, m_recvBuffer, size, PacketChannel::Control);
            ++processed;
            if (processed >= MAX_CONTROL_PACKETS_PER_UPDATE)
                break;
        }
    }

    void NetworkingSystem::UpdateReliableResendsOnControlSocket(float dt)
    {
        constexpr float RESEND_INTERVAL_SEC = 0.05f;
        constexpr int   MAX_RETRIES = 10;

        for (auto& peer : m_peers)
        {
            for (size_t i = 0; i < peer.resendQueue.size();)
            {
                auto& msg = peer.resendQueue[i];
                msg.timer += dt;

                if (msg.timer >= RESEND_INTERVAL_SEC)
                {
                    if (msg.retries >= MAX_RETRIES)
                    {
                        peer.resendQueue.erase(peer.resendQueue.begin() + (int)i);
                        continue;
                    }

                    m_controlSocket.Send(
                        peer.controlAddr,
                        peer.controlAddrLen,
                        msg.data.data(),
                        (int)msg.data.size());

                    ++m_stats.reliableResends;
                    msg.timer = 0.0f;
                    msg.retries++;
                }

                ++i;
            }
        }
    }

    void NetworkingSystem::ReceiveSnapshotPacketsWithBudget()
    {
        sockaddr_storage from{};
        int size = 0;
        int processed = 0;
        while ((size = m_snapshotSocket.Receive(from, m_recvBuffer, (int)sizeof(m_recvBuffer))) > 0)
        {
            HandlePacket(from, m_recvBuffer, size, PacketChannel::Snapshot);
            ++processed;
            if (processed >= MAX_SNAPSHOT_PACKETS_PER_UPDATE)
                break;
        }
    }

    void NetworkingSystem::DeliverDelayedOutgoingSnapshotsWithBudget(float dt)
    {
        if (m_snapshotPauseSeconds > 0.0f)
        {
            m_delayedOutgoingSnapshots.clear();
            return;
        }

        int sentThisUpdate = 0;
        for (size_t i = 0; i < m_delayedOutgoingSnapshots.size();)
        {
            auto& p = m_delayedOutgoingSnapshots[i];
            p.delaySec -= dt;

            if (p.delaySec <= 0.0f)
            {
                m_snapshotSocket.Send(
                    p.addr,
                    p.addrLen,
                    p.payload.data(),
                    (int)p.payload.size());

                m_delayedOutgoingSnapshots.erase(
                    m_delayedOutgoingSnapshots.begin() + (int)i);

                ++sentThisUpdate;
                if (sentThisUpdate >= MAX_DELAYED_SNAPSHOT_SENDS_PER_UPDATE)
                    break;
                continue;
            }

            ++i;
        }
    }

    void NetworkingSystem::DeliverDelayedIncomingSnapshotsIfStillUsed(float dt)
    {
        int delivered = 0;
        for (size_t i = 0; i < m_delayedIncomingSnapshots.size();)
        {
            auto& p = m_delayedIncomingSnapshots[i];
            p.delaySec -= dt;

            if (p.delaySec <= 0.0f)
            {
                ReceivedSnapshotChunk chunk{};
                chunk.tick = p.tick;
                chunk.items = std::move(p.items);

                m_pendingSnapshotChunks.push_back(std::move(chunk));
                m_delayedIncomingSnapshots.erase(
                    m_delayedIncomingSnapshots.begin() + (int)i);

                ++delivered;
                if (delivered >= MAX_SNAPSHOT_PACKETS_PER_UPDATE)
                    break;
                continue;
            }

            ++i;
        }
    }

    void NetworkingSystem::HandlePacket(
        const sockaddr_storage& from,
        const char* data,
        int size,
        PacketChannel channel)
    {
        if (size < (int)sizeof(MsgHeader))
            return;

        const MsgHeader* hdr = reinterpret_cast<const MsgHeader*>(data);

        if (hdr->protocolVersion != PROTOCOL_VERSION)
            return;

        if (hdr->peerId == 0)
            return;

        Peer* peer = nullptr;
        if (channel == PacketChannel::Control)
            peer = FindPeerByControlAddr(m_peers, from);
        else
            peer = FindPeerBySnapshotAddr(m_peers, from);

        if (!peer)
        {
            peer = FindPeerById(m_peers, (int)hdr->peerId);
            if (!peer)
                return;

            if (channel == PacketChannel::Control)
            {
                peer->controlAddr = from;
                peer->controlAddrLen = Net::SockaddrLen(from);
            }
            else
            {
                peer->snapshotAddr = from;
                peer->snapshotAddrLen = Net::SockaddrLen(from);
            }
        }
        peer->active = true;

        const MsgType msgType = (MsgType)hdr->msgType;
        if (channel == PacketChannel::Control && msgType == MsgType::STATE_SNAPSHOT)
            return;
        if (channel == PacketChannel::Snapshot && msgType != MsgType::STATE_SNAPSHOT)
            return;

        switch (msgType)
        {
        case MsgType::HELLO:
        {
            std::cout << "Received HELLO from peer " << (int)hdr->peerId << "\n";

            MsgHeader reply{};
            reply.msgType = (uint8_t)MsgType::WELCOME;
            reply.peerId = (uint8_t)m_localPeerId;
            reply.seq = peer->nextSeq++;

            m_controlSocket.Send(peer->controlAddr, peer->controlAddrLen, &reply, (int)sizeof(reply));
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

            queue.erase(
                std::remove_if(
                    queue.begin(),
                    queue.end(),
                    [&](const PendingMessage& m)
                    {
                        return m.seq == ackSeq;
                    }),
                queue.end());

            break;
        }

        case MsgType::GLOBAL_COMMAND:
        {
            // Reliable channel: always ACK, including duplicates.
            SendAck(m_controlSocket, *peer, m_localPeerId, hdr->seq);

            if (size < (int)sizeof(MsgHeader) + (int)sizeof(GlobalCommandPayload))
                break;

            // Duplicate suppression without assuming in-order UDP delivery.
            if (IsDuplicateReliable(*peer, hdr->seq))
                break;

            MarkReliableReceived(*peer, hdr->seq);

            ++m_stats.globalCommandsReceived;

            const GlobalCommandPayload* payload =
                reinterpret_cast<const GlobalCommandPayload*>(
                    data + sizeof(MsgHeader));

            const auto commandType =
                (GlobalCommandType)payload->commandType;

            if (commandType == GlobalCommandType::SceneChange)
            {
                // A scene change invalidates all object traffic from the old scene.
                ClearSceneObjectTraffic();
                PauseSnapshotTraffic(0.25f);

                // Important:
                // m_currentSceneGeneration is set here so packets from the new scene
                // can be accepted by the networking layer as soon as this command arrives.
                //
                // The main shared sceneGeneration should also be updated from this payload
                // wherever you process PopReceivedGlobalCommand().
                m_currentSceneGeneration = payload->sceneGeneration;
            }
            else if (commandType == GlobalCommandType::GravityOnOff)
            {
                PauseSnapshotTraffic(0.10f);
            }

            // Store latest command for PopReceivedGlobalCommand().
            m_pendingGlobal = *payload;
            m_hasPendingGlobal.store(true, std::memory_order_release);

            break;
        }

        case MsgType::STATE_SNAPSHOT:
        {
            // Snapshots are low-priority and disposable.
            // During scene/global transitions, old snapshots are stale.
            if (m_snapshotPauseSeconds > 0.0f)
            {
                ++m_stats.snapshotPacketsDropped;
                break;
            }

            if (size < (int)(sizeof(MsgHeader) + sizeof(StateSnapshotHeader)))
                break;

            const StateSnapshotHeader* sh =
                reinterpret_cast<const StateSnapshotHeader*>(
                    data + sizeof(MsgHeader));

            // Drop snapshots from the wrong scene instance.
            // This prevents old-scene snapshots from corrupting the current scene.
            if (sh->sceneGeneration != m_currentSceneGeneration)
            {
                ++m_stats.snapshotPacketsDropped;
                break;
            }

            const int count = (int)sh->count;

            if (count <= 0)
                break;

            const int expectedSize =
                (int)sizeof(MsgHeader) +
                (int)sizeof(StateSnapshotHeader) +
                count * (int)sizeof(StateSnapshotItem);

            if (expectedSize > size)
                break;

            ++m_stats.snapshotPacketsReceived;

            const StateSnapshotItem* items =
                reinterpret_cast<const StateSnapshotItem*>(
                    data + sizeof(MsgHeader) + sizeof(StateSnapshotHeader));

            // Do NOT apply artificial latency/drop here.
            // Impairment is outgoing-only in SendStateSnapshot().
            ReceivedSnapshotChunk chunk{};
            chunk.tick = hdr->tick;
            chunk.items.assign(items, items + count);

            m_pendingSnapshotChunks.push_back(std::move(chunk));

            break;
        }

        case MsgType::SPAWN_OBJECT:
        {
            // Reliable channel: always ACK, including duplicates.
            SendAck(m_controlSocket, *peer, m_localPeerId, hdr->seq);

            if (size < (int)sizeof(MsgHeader) + (int)sizeof(SpawnObjectPayload))
                break;

            // Duplicate suppression without assuming in-order UDP delivery.
            if (IsDuplicateReliable(*peer, hdr->seq))
                break;

            MarkReliableReceived(*peer, hdr->seq);

            const SpawnObjectPayload* payload =
                reinterpret_cast<const SpawnObjectPayload*>(
                    data + sizeof(MsgHeader));

            // Drop spawns from the wrong scene instance.
            // ACK has already been sent, so the sender stops resending it,
            // but this peer will not apply stale object creation.
            if (payload->sceneGeneration != m_currentSceneGeneration)
            {
                break;
            }

            m_pendingSpawnObjects.push_back(*payload);
            ++m_stats.spawnPacketsReceived;

            break;
        }

        case MsgType::PING:
        {
            if (size < (int)sizeof(MsgHeader) + (int)sizeof(PingPayload))
                break;

            struct PongPacket
            {
                MsgHeader header;
                PingPayload payload;
            } reply{};

            reply.header.msgType = (uint8_t)MsgType::PONG;
            reply.header.peerId = (uint8_t)m_localPeerId;
            reply.payload = *reinterpret_cast<const PingPayload*>(data + sizeof(MsgHeader));

            m_controlSocket.Send(
                peer->controlAddr,
                peer->controlAddrLen,
                &reply,
                (int)sizeof(reply));
            break;
        }

        case MsgType::PONG:
        {
            if (size < (int)sizeof(MsgHeader) + (int)sizeof(PingPayload))
                break;

            const PingPayload* payload =
                reinterpret_cast<const PingPayload*>(data + sizeof(MsgHeader));

            const auto pending = peer->pendingPings.find(payload->pingId);
            if (pending == peer->pendingPings.end())
                break;

            const double nowSec = NowSeconds();
            const double rttMs = std::max(0.0, (nowSec - pending->second) * 1000.0);
            peer->pendingPings.erase(pending);

            const double oldAvg = peer->avgRttMs;
            peer->lastRttMs = rttMs;
            peer->avgRttMs = oldAvg < 0.0 ? rttMs : oldAvg + (rttMs - oldAvg) * 0.15;
            peer->jitterMs = oldAvg < 0.0 ? 0.0 : peer->jitterMs + (std::abs(rttMs - oldAvg) - peer->jitterMs) * 0.15;
            ++peer->pongsReceived;
            break;
        }

        case MsgType::DISCOVER_SIM_PEER:
        {
            if (size < (int)sizeof(MsgHeader) + (int)sizeof(DiscoveryPayload))
                break;

            const DiscoveryPayload* payload =
                reinterpret_cast<const DiscoveryPayload*>(data + sizeof(MsgHeader));

            peer->controlAddr = AddressWithPort(from, payload->controlPort);
            peer->controlAddrLen = Net::SockaddrLen(peer->controlAddr);
            peer->snapshotAddr = AddressWithPort(from, payload->snapshotPort);
            peer->snapshotAddrLen = Net::SockaddrLen(peer->snapshotAddr);
            peer->active = true;
            SendHelloToPeer(m_controlSocket, *peer, m_localPeerId);
            ++m_stats.discoveryPacketsReceived;
            ++m_stats.peersDiscovered;

            struct DiscoveryPacket
            {
                MsgHeader header;
                DiscoveryPayload payload;
            } reply{};

            reply.header.msgType = (uint8_t)MsgType::PEER_HERE;
            reply.header.peerId = (uint8_t)m_localPeerId;
            reply.payload.peerId = (uint8_t)m_localPeerId;
            reply.payload.controlPort = m_controlBindPort;
            reply.payload.snapshotPort = m_snapshotBindPort;

            m_controlSocket.Send(
                peer->controlAddr,
                peer->controlAddrLen,
                &reply,
                (int)sizeof(reply));
            break;
        }

        case MsgType::PEER_HERE:
        {
            if (size < (int)sizeof(MsgHeader) + (int)sizeof(DiscoveryPayload))
                break;

            const DiscoveryPayload* payload =
                reinterpret_cast<const DiscoveryPayload*>(data + sizeof(MsgHeader));

            peer->controlAddr = AddressWithPort(from, payload->controlPort);
            peer->controlAddrLen = Net::SockaddrLen(peer->controlAddr);
            peer->snapshotAddr = AddressWithPort(from, payload->snapshotPort);
            peer->snapshotAddrLen = Net::SockaddrLen(peer->snapshotAddr);
            peer->active = true;
            SendHelloToPeer(m_controlSocket, *peer, m_localPeerId);
            ++m_stats.discoveryPacketsReceived;
            ++m_stats.peersDiscovered;
            break;
        }

        default:
            break;
        }
    }

    // ============================================================
    // GLOBAL_COMMAND sending
    // ============================================================
    void NetworkingSystem::SetCurrentSceneGeneration(uint32_t generation)
    {
        m_currentSceneGeneration = generation;
    }


    void NetworkingSystem::SendSceneChange(int sceneIndex, uint32_t sceneGeneration)
    {
        ClearSceneObjectTraffic();
        PauseSnapshotTraffic(0.25f);

        // Local networking layer should immediately treat the new scene generation
        // as current, so it does not send/accept old-scene object traffic.
        m_currentSceneGeneration = sceneGeneration;

        GlobalCommandPayload p{};
        p.commandType = (uint8_t)GlobalCommandType::SceneChange;
        p.sceneIndex = sceneIndex;
        p.sceneGeneration = sceneGeneration;

        SendGlobalCommand(p);
        SendGlobalCommand(p);
        SendGlobalCommand(p);
    }

    void NetworkingSystem::SendGravityEnabled(bool enabled)
    {
        PauseSnapshotTraffic(0.10f);

        GlobalCommandPayload p{};
        p.commandType = (uint8_t)GlobalCommandType::GravityOnOff;
        p.gravityEnabled = enabled ? 1 : 0;

        // Redundant send is cheap and makes the global UI feel immediate.
        SendGlobalCommand(p);
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

            SendReliable(
                m_controlSocket,
                peer,
                buffer,
                (int)(sizeof(hdr) + sizeof(payload)),
                true);

            ++m_stats.globalCommandsSent;
        }
    }

    // ============================================================
    // SPAWN_OBJECT sending / receiving
    // ============================================================

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

            SendReliable(
                m_controlSocket,
                peer,
                buffer,
                (int)(sizeof(hdr) + sizeof(payload)),
                false);
            ++m_stats.spawnPacketsSent;
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
    // STATE_SNAPSHOT receiving
    // ============================================================

    bool NetworkingSystem::PopReceivedStateSnapshot(
        std::vector<StateSnapshotItem>& outItems,
        uint32_t& outTick)
    {
        if (m_pendingSnapshotChunks.empty())
            return false;

        ReceivedSnapshotChunk chunk = std::move(m_pendingSnapshotChunks.front());
        m_pendingSnapshotChunks.pop_front();

        outTick = chunk.tick;
        outItems = std::move(chunk.items);

        return true;
    }

    // ============================================================
    // Network stats
    // ============================================================

    Net::NetworkStats NetworkingSystem::GetStats() const
    {
        NetworkStats stats = m_stats;

        stats.delayedOutgoingSnapshotPackets =
            static_cast<uint32_t>(m_delayedOutgoingSnapshots.size());

        stats.delayedIncomingSnapshotPackets =
            static_cast<uint32_t>(m_delayedIncomingSnapshots.size());

        return stats;
    }

    std::vector<int> NetworkingSystem::GetActivePeerIds() const
    {
        std::vector<int> ids;
        for (const auto& peer : m_peers)
        {
            if (peer.active)
                ids.push_back(peer.peerId);
        }
        return ids;
    }

    std::vector<PeerDebugInfo> NetworkingSystem::GetPeerDebugInfo() const
    {
        std::vector<PeerDebugInfo> info;
        info.reserve(m_peers.size());

        for (const auto& peer : m_peers)
        {
            PeerDebugInfo row{};
            row.peerId = peer.peerId;
            row.active = peer.active;
            row.lastRttMs = peer.lastRttMs;
            row.avgRttMs = peer.avgRttMs;
            row.jitterMs = peer.jitterMs;
            row.pingsSent = peer.pingsSent;
            row.pongsReceived = peer.pongsReceived;
            row.pingsTimedOut = peer.pingsTimedOut;
            row.pendingPings = (uint32_t)peer.pendingPings.size();
            info.push_back(row);
        }

        return info;
    }

    // ============================================================
    // STATE_SNAPSHOT sending
    //
    // Snapshots are unreliable, low-priority, chunked, and budgeted.
    // ============================================================

    void NetworkingSystem::SendStateSnapshot(
        uint32_t tick,
        uint32_t sceneGeneration,
        const StateSnapshotItem* items,
        uint32_t count)
    {
        if (!items || count == 0)
            return;

        // Snapshots are low priority. During scene/global transitions, skip them.
        if (m_snapshotPauseSeconds > 0.0f)
            return;

        // Conservative UDP payload size to avoid fragmentation.
        static constexpr int SAFE_UDP_PACKET_BYTES = 1200;

        const int headerBytes =
            (int)sizeof(MsgHeader) +
            (int)sizeof(StateSnapshotHeader);

        const int maxItemsPerPacket =
            (SAFE_UDP_PACKET_BYTES - headerBytes) /
            (int)sizeof(StateSnapshotItem);

        if (maxItemsPerPacket <= 0)
        {
            std::cout << "[NET] STATE_SNAPSHOT item too large for packet\n";
            return;
        }

        const uint32_t totalChunks =
            (count + (uint32_t)maxItemsPerPacket - 1u) /
            (uint32_t)maxItemsPerPacket;

        if (totalChunks == 0)
            return;

        if (totalChunks > UINT16_MAX)
        {
            std::cout << "[NET] STATE_SNAPSHOT too many chunks: "
                << totalChunks
                << " for count="
                << count
                << "\n";
            return;
        }

        // Do not send the whole snapshot if it is huge.
        // Send a rotating window so reliable control traffic stays responsive.
        const uint32_t chunksToSend =
            std::min<uint32_t>(totalChunks, m_maxSnapshotPacketsPerSend);

        char buffer[SAFE_UDP_PACKET_BYTES];

        for (uint32_t sent = 0; sent < chunksToSend; ++sent)
        {
            const uint32_t chunkIndex =
                (m_snapshotSendCursor + sent) % totalChunks;

            const uint32_t firstItem =
                chunkIndex * (uint32_t)maxItemsPerPacket;

            const uint32_t remaining = count - firstItem;

            const uint32_t itemsInChunk =
                std::min<uint32_t>(
                    remaining,
                    (uint32_t)maxItemsPerPacket);

            if (itemsInChunk == 0)
                continue;

            const int bytes =
                headerBytes +
                (int)itemsInChunk * (int)sizeof(StateSnapshotItem);

            MsgHeader hdr{};
            hdr.msgType = (uint8_t)MsgType::STATE_SNAPSHOT;
            hdr.peerId = (uint8_t)m_localPeerId;
            hdr.tick = tick;

            StateSnapshotHeader sh{};
            sh.count = (uint16_t)itemsInChunk;
            sh.chunkIndex = (uint16_t)chunkIndex;
            sh.chunkCount = (uint16_t)totalChunks;
            sh.sceneGeneration = sceneGeneration;

            std::memcpy(buffer, &hdr, sizeof(hdr));
            std::memcpy(buffer + sizeof(hdr), &sh, sizeof(sh));
            std::memcpy(
                buffer + sizeof(hdr) + sizeof(sh),
                items + firstItem,
                itemsInChunk * sizeof(StateSnapshotItem));

            for (auto& peer : m_peers)
            {
                if (ShouldDropSnapshotPacket())
                {
                    ++m_stats.snapshotPacketsDropped;
                    continue;
                }

                const float delaySec = SampleSnapshotDelaySeconds();

                if (delaySec > 0.0f)
                {
                    if (m_delayedOutgoingSnapshots.size() >= MAX_DELAYED_OUTGOING_SNAPSHOTS)
                    {
                        ++m_stats.snapshotPacketsDropped;
                        continue;
                    }

                    DelayedOutgoingSnapshot delayed{};
                    delayed.addr = peer.snapshotAddr;
                    delayed.addrLen = peer.snapshotAddrLen;
                    delayed.payload.assign(buffer, buffer + bytes);
                    delayed.delaySec = delaySec;

                    m_delayedOutgoingSnapshots.push_back(std::move(delayed));

                    ++m_stats.snapshotPacketsDelayed;
                }
                else
                {
                    m_snapshotSocket.Send(peer.snapshotAddr, peer.snapshotAddrLen, buffer, bytes);
                }

                ++m_stats.snapshotPacketsSent;
            }
        }

        m_snapshotSendCursor =
            (m_snapshotSendCursor + chunksToSend) % totalChunks;
    }

    // ============================================================
    // Snapshot impairment
    // ============================================================

    void NetworkingSystem::SetSnapshotImpairment(
        const SnapshotImpairmentSettings& settings)
    {
        m_snapshotImpairment.enabled = settings.enabled;
        m_snapshotImpairment.latencyMs = std::max(0.0f, settings.latencyMs);
        m_snapshotImpairment.jitterMs = std::max(0.0f, settings.jitterMs);
        m_snapshotImpairment.dropPercent =
            std::max(0.0f, std::min(settings.dropPercent, 100.0f));
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
            const float jitter =
                (m_unitDist(m_rng) * 2.0f - 1.0f) * jitterMs;

            delayMs += jitter;
        }

        delayMs = std::max(0.0f, delayMs);
        return delayMs * 0.001f;
    }

    // ============================================================
    // Snapshot priority control
    // ============================================================

    void NetworkingSystem::ClearSnapshotBacklog()
    {
        m_delayedOutgoingSnapshots.clear();
        m_delayedIncomingSnapshots.clear();
        m_pendingSnapshotChunks.clear();

        m_snapshotSendCursor = 0;
    }

    void NetworkingSystem::PauseSnapshotTraffic(float seconds)
    {
        m_snapshotPauseSeconds =
            std::max(m_snapshotPauseSeconds, seconds);
    }

    void NetworkingSystem::ClearSceneObjectTraffic()
    {
        ClearSnapshotBacklog();

        m_pendingSpawnObjects.clear();

        for (auto& peer : m_peers)
        {
            auto& q = peer.resendQueue;

            q.erase(
                std::remove_if(
                    q.begin(),
                    q.end(),
                    [](const PendingMessage& msg)
                    {
                        return PendingMessageIsType(msg, MsgType::SPAWN_OBJECT);
                    }),
                q.end());
        }
    }
}
