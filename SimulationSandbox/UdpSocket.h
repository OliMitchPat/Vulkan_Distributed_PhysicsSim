#pragma once
#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>
#include <cstdint>

namespace Net
{
    class UdpSocket
    {
    public:
        bool Open(const std::string& bindIp, uint16_t port, std::string& error);
        void Close();

        bool Send(const sockaddr_storage& addr, int addrLen, const void* data, int size);

        int Receive(sockaddr_storage& from, void* buffer, int maxSize);

    private:
        SOCKET m_socket = INVALID_SOCKET;
    };
}