#include "NetworkingSystem.h"
#include "NetAddress.h"
#include <iostream>
#include <cstring>
#include <algorithm>

namespace Net
{
    // ============================================================
    // Internal helper: find peer by address
    // ============================================================
    static Peer* FindPeerById(std::vector<Peer>& peers, int peerId)
    {
        for (auto& p : peers)
            if (p.peerId == peerId)
                return &p;
        return nullptr;
    }

    // ============================================================
    // Init
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

        for (const auto& p : cfg.RemotePeers())
        {
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

            m_peers.push_back(peer);
        }

        SendHello();
        return true;
    }

    // ============================================================
    // Shutdown
    // ============================================================
    void NetworkingSystem::Shutdown()
    {
        m_socket.Close();
    }

    // ============================================================
    // Send HELLO (unreliable is fine)
    // ============================================================
    void NetworkingSystem::SendHello()
    {
        MsgHeader msg{};
        msg.msgType = (uint8_t)MsgType::HELLO;
        msg.peerId = (uint8_t)m_localPeerId;

        for (auto& p : m_peers)
        {
            m_socket.Send(p.addr, p.addrLen, &msg, sizeof(msg));
        }
    }

    // ============================================================
    // Send ACK
    // ============================================================
    void SendAck(UdpSocket& socket, Peer& peer, int localPeerId, uint32_t seq)
    {
        MsgHeader ack{};
        ack.msgType = (uint8_t)MsgType::ACK;
        ack.peerId = (uint8_t)localPeerId;
        ack.ack = seq;

        socket.Send(peer.addr, peer.addrLen, &ack, sizeof(ack));
    }

    // ============================================================
// Send Reliable Message
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
    // Update
    // ============================================================
    void NetworkingSystem::Update(float dt)
    {
        sockaddr_storage from{};

        // ---- Receive loop ----
        int size = 0;
        while ((size = m_socket.Receive(from, m_recvBuffer, (int)sizeof(m_recvBuffer))) > 0)
        {
            HandlePacket(from, m_recvBuffer, size);
        }

        // ---- Resend logic ----
        constexpr float RESEND_INTERVAL_SEC = 0.5f;
        constexpr int   MAX_RETRIES = 10;

        for (auto& peer : m_peers)
        {
            // Iterate with index so we can erase safely
            for (size_t i = 0; i < peer.resendQueue.size(); /* increment inside */)
            {
                auto& msg = peer.resendQueue[i];
                msg.timer += dt;

                if (msg.timer >= RESEND_INTERVAL_SEC)
                {
                    if (msg.retries >= MAX_RETRIES)
                    {
                        // Give up on this message
                        std::cout << "[NET] Drop reliable msg seq=" << msg.seq
                            << " to peer " << peer.peerId
                            << " (max retries reached)\n";

                        peer.resendQueue.erase(peer.resendQueue.begin() + i);
                        continue; // don't increment i (we erased current)
                    }

                    // Resend
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

        // Basic sanity: ignore invalid/self
        if (hdr->peerId == 0 || (int)hdr->peerId == m_localPeerId)
            return;

        Peer* peer = FindPeerById(m_peers, (int)hdr->peerId);
        if (!peer)
            return;

        // Learn/update the sender address so ACKs/resends go to the correct endpoint.
        // (This is important now that we no longer match peers by address.)
        peer->addr = from;
        peer->addrLen = Net::SockaddrLen(from);

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
            auto& queue = peer->resendQueue;

            queue.erase(
                std::remove_if(queue.begin(), queue.end(),
                    [&](const PendingMessage& m)
                    {
                        return m.seq == hdr->ack;
                    }),
                queue.end());

            break;
        }

        case MsgType::GLOBAL_COMMAND:
        {
            // Always ACK reliable messages (even duplicates)
            SendAck(m_socket, *peer, m_localPeerId, hdr->seq);

            // Duplicate suppression (prevents applying the same reliable command twice)
            if (hdr->seq != 0 && hdr->seq <= peer->lastReceivedSeq)
                break;

            peer->lastReceivedSeq = hdr->seq;

            std::cout << "Received GLOBAL_COMMAND from peer " << (int)hdr->peerId << "\n";

            if (size < (int)sizeof(MsgHeader) + (int)sizeof(GlobalCommandPayload))
                break;

            const GlobalCommandPayload* payload =
                reinterpret_cast<const GlobalCommandPayload*>(data + sizeof(MsgHeader));

            switch ((GlobalCommandType)payload->commandType)
            {
            case GlobalCommandType::ToggleGravity:
                std::cout << "GLOBAL: Toggle Gravity\n";
                // later: hook into physics system
                break;

            case GlobalCommandType::ResetScene:
                std::cout << "GLOBAL: Reset Scene\n";
                break;

            default:
                break;
            }

            break;
        }

        default:
            break;
        }
    }

    void NetworkingSystem::SendGlobalCommand(uint8_t commandType)
    {
        for (auto& peer : m_peers)
        {
            MsgHeader hdr{};
            hdr.msgType = (uint8_t)MsgType::GLOBAL_COMMAND;
            hdr.peerId = (uint8_t)m_localPeerId;
            hdr.seq = peer.nextSeq++;

            GlobalCommandPayload payload{};
            payload.commandType = commandType;

            char buffer[256];
            memcpy(buffer, &hdr, sizeof(hdr));
            memcpy(buffer + sizeof(hdr), &payload, sizeof(payload));

            SendReliable(m_socket, peer, buffer, sizeof(hdr) + sizeof(payload));
        }
    }
} 