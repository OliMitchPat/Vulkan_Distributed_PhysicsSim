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
    static Peer* FindPeer(std::vector<Peer>& peers, const sockaddr_storage& addr)
    {
        for (auto& p : peers)
        {
            if (memcmp(&p.addr, &addr, sizeof(sockaddr_in)) == 0)
                return &p;
        }
        return nullptr;
    }

    // ============================================================
    // Init
    // ============================================================
    bool NetworkingSystem::Init(const PeerConfig& cfg)
    {
        std::string err;
        if (!m_socket.Open(cfg.bind_port, err))
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
            m_socket.Send(p.addr, &msg, sizeof(msg));
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

        socket.Send(peer.addr, &ack, sizeof(ack));
    }

    // ============================================================
    // Send Reliable Message
    // ============================================================
    void SendReliable(UdpSocket& socket, Peer& peer, const void* data, int size)
    {
        socket.Send(peer.addr, data, size);

        PendingMessage msg;
        msg.data.assign((const char*)data, (const char*)data + size);
        msg.seq = ((const MsgHeader*)data)->seq;
        msg.timer = 0.0f;

        peer.resendQueue.push_back(msg);
    }

    // ============================================================
    // Update
    // ============================================================
    void NetworkingSystem::Update()
    {
        sockaddr_storage from{};

        // ---- Receive loop ----
        int size = 0;
        while ((size = m_socket.Receive(from, m_recvBuffer, sizeof(m_recvBuffer))) > 0)
        {
            HandlePacket(from, m_recvBuffer, size);
        }

        // ---- Resend logic ----
        const float dt = 1.0f / 60.0f; // simple for now

        for (auto& peer : m_peers)
        {
            for (auto& msg : peer.resendQueue)
            {
                msg.timer += dt;

                if (msg.timer > 0.5f)
                {
                    m_socket.Send(peer.addr, msg.data.data(), (int)msg.data.size());
                    msg.timer = 0.0f;
                }
            }
        }
    }

    // ============================================================
    // Handle incoming packet
    // ============================================================
    void NetworkingSystem::HandlePacket(const sockaddr_storage& from, const char* data, int size)
    {
        if (size < sizeof(MsgHeader)) return;

        const MsgHeader* hdr = (const MsgHeader*)data;

        if (hdr->protocolVersion != PROTOCOL_VERSION)
            return;

        Peer* peer = FindPeer(m_peers, from);
        if (!peer)
            return;

        switch ((MsgType)hdr->msgType)
        {
        case MsgType::HELLO:
        {
            std::cout << "Received HELLO from peer " << (int)hdr->peerId << "\n";

            MsgHeader reply{};
            reply.msgType = (uint8_t)MsgType::WELCOME;
            reply.peerId = (uint8_t)m_localPeerId;
            reply.seq = peer->nextSeq++;

            m_socket.Send(from, &reply, sizeof(reply));
            break;
        }

        case MsgType::WELCOME:
        {
            std::cout << "Received WELCOME from peer " << (int)hdr->peerId << "\n";
            break;
        }

        case MsgType::ACK:
        {
            // ?? Remove acknowledged message
            auto& queue = peer->resendQueue;

            queue.erase(
                std::remove_if(queue.begin(), queue.end(),
                    [&](const PendingMessage& m)
                    {
                        return m.seq == hdr->ack;
                    }),
                queue.end()
            );

            break;
        }

        case MsgType::GLOBAL_COMMAND:
        {
            std::cout << "Received GLOBAL_COMMAND from peer " << (int)hdr->peerId << "\n";

            //  Always ACK reliable messages
            SendAck(m_socket, *peer, m_localPeerId, hdr->seq);

            //  Make sure payload exists
            if (size < sizeof(MsgHeader) + sizeof(GlobalCommandPayload))
                break;

            //  Read payload
            const GlobalCommandPayload* payload =
                (const GlobalCommandPayload*)(data + sizeof(MsgHeader));

            //  Handle command
            switch ((GlobalCommandType)payload->commandType)
            {
            case GlobalCommandType::ToggleGravity:
            {
                std::cout << "GLOBAL: Toggle Gravity\n";
                // later: hook into physics system
                break;
            }

            case GlobalCommandType::ResetScene:
            {
                std::cout << "GLOBAL: Reset Scene\n";
                break;
            }

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