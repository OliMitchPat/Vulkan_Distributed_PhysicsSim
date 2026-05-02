#pragma once
#include "UdpSocket.h"
#include "NetProtocol.h"
#include "NetPeer.h"
#include "PeerConfig.h"

#include <vector>
#include <unordered_map>

namespace Net
{
    class NetworkingSystem
    {
    public:
        bool Init(const PeerConfig& cfg);
        void Shutdown();

        void Update(float dt);
        void SendGlobalCommand(uint8_t commandType);
    private:
        void SendHello();
        void HandlePacket(const sockaddr_storage& from, const char* data, int size);


    private:
        UdpSocket m_socket;

        int m_localPeerId = 0;

        std::vector<Peer> m_peers;

        char m_recvBuffer[1500];
    };
}