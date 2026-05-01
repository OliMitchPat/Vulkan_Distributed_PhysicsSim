#include "UdpSocket.h"

namespace Net
{
    bool UdpSocket::Open(uint16_t port, std::string& error)
    {
        m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (m_socket == INVALID_SOCKET)
        {
            error = "socket() failed";
            return false;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(m_socket, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR)
        {
            error = "bind() failed";
            return false;
        }

        u_long nonBlocking = 1;
        ioctlsocket(m_socket, FIONBIO, &nonBlocking);

        return true;
    }

    void UdpSocket::Close()
    {
        if (m_socket != INVALID_SOCKET)
        {
            closesocket(m_socket);
            m_socket = INVALID_SOCKET;
        }
    }

    bool UdpSocket::Send(const sockaddr_storage& addr, const void* data, int size)
    {
        int sent = sendto(m_socket,
            (const char*)data,
            size,
            0,
            (const sockaddr*)&addr,
            sizeof(addr));

        return sent == size;
    }

    int UdpSocket::Receive(sockaddr_storage& from, void* buffer, int maxSize)
    {
        int fromLen = sizeof(from);

        int result = recvfrom(m_socket,
            (char*)buffer,
            maxSize,
            0,
            (sockaddr*)&from,
            &fromLen);

        if (result == SOCKET_ERROR)
        {
            int err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK)
                return 0;

            return -1;
        }

        return result;
    }
}