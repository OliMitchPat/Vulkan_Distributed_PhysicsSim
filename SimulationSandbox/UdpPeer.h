#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <vector>
#include <string>
#include <unordered_map>
#include <cstdint>

#include "PeerConfig.h"
#include "NetAddress.h"

#pragma comment(lib, "Ws2_32.lib")

namespace Net
{
    struct PeerInfo
    {
        int peerId = 0;
        sockaddr_storage addr{};
    };

    class UdpPeer
    {
    public:
        bool Init(const PeerConfig& cfg, std::string& err)
        {
            m_peerId = cfg.peer_id;

            // --- WSA Startup ---
            WSADATA wsa{};
            if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
            {
                err = "WSAStartup failed";
                return false;
            }

            // --- Create socket ---
            m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (m_socket == INVALID_SOCKET)
            {
                err = "socket() failed";
                return false;
            }

            // --- Non-blocking ---
            u_long mode = 1;
            ioctlsocket(m_socket, FIONBIO, &mode);

            // --- Bind ---
            sockaddr_storage bindAddr{};
            if (!ResolveAddress(cfg.bind_ip, cfg.bind_port, bindAddr, err))
                return false;

            if (bind(m_socket,
                reinterpret_cast<sockaddr*>(&bindAddr),
                sizeof(bindAddr)) != 0)
            {
                err = "bind() failed";
                return false;
            }

            // --- Build peer table ---
            for (const auto& p : cfg.peers)
            {
                sockaddr_storage addr{};
                if (!ResolveAddress(p.host, p.port, addr, err))
                    return false;

                PeerInfo info;
                info.peerId = p.peerId;
                info.addr = addr;

                m_peers[p.peerId] = info;
            }

            return true;
        }

        void Shutdown()
        {
            if (m_socket != INVALID_SOCKET)
            {
                closesocket(m_socket);
                m_socket = INVALID_SOCKET;
            }
            WSACleanup();
        }

        void SendTo(int peerId, const void* data, int size)
        {
            auto it = m_peers.find(peerId);
            if (it == m_peers.end()) return;

            const auto& addr = it->second.addr;

            sendto(m_socket,
                reinterpret_cast<const char*>(data),
                size,
                0,
                reinterpret_cast<const sockaddr*>(&addr),
                sizeof(addr));
        }

        void Broadcast(const void* data, int size)
        {
            for (const auto& [id, peer] : m_peers)
            {
                if (id == m_peerId) continue; // skip self
                SendTo(id, data, size);
            }
        }

        void PollReceive()
        {
            char buffer[1500];

            while (true)
            {
                sockaddr_storage from{};
                int fromLen = sizeof(from);

                int bytes = recvfrom(
                    m_socket,
                    buffer,
                    sizeof(buffer),
                    0,
                    reinterpret_cast<sockaddr*>(&from),
                    &fromLen);

                if (bytes <= 0)
                    break; // no more packets (non-blocking)

                // TEMP: just print
                std::string addrStr = AddressToString(from);
                printf("[NET] Received %d bytes from %s\n", bytes, addrStr.c_str());
            }
        }

    private:
        SOCKET m_socket = INVALID_SOCKET;
        int m_peerId = 0;
        std::unordered_map<int, PeerInfo> m_peers;
    };
}