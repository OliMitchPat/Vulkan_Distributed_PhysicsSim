#include "UdpSocket.h"
#include "NetAddress.h" // ResolveAddress + SockaddrLen

#include <iostream>

#ifdef _WIN32
#include <ws2tcpip.h> // inet_ntop
#endif

namespace Net
{
    static std::string SockaddrToString(const sockaddr_storage& ss)
    {
        if (ss.ss_family == AF_INET)
        {
            const auto* in = reinterpret_cast<const sockaddr_in*>(&ss);
            char ip[INET_ADDRSTRLEN] = {};
            inet_ntop(AF_INET, (void*)&in->sin_addr, ip, sizeof(ip));
            return std::string(ip) + ":" + std::to_string(ntohs(in->sin_port));
        }
        return "<non-ipv4>";
    }

    bool UdpSocket::Open(const std::string& bindIp, uint16_t port, std::string& error)
    {
        sockaddr_storage bindAddr{};
        if (!ResolveAddress(bindIp, port, bindAddr, error))
            return false;

        const int family = bindAddr.ss_family;

        m_socket = socket(family, SOCK_DGRAM, IPPROTO_UDP);
        if (m_socket == INVALID_SOCKET)
        {
            error = "socket() failed WSA=" + std::to_string(WSAGetLastError());
            return false;
        }

        const int bindLen = SockaddrLen(bindAddr);
        if (bind(m_socket, reinterpret_cast<const sockaddr*>(&bindAddr), bindLen) == SOCKET_ERROR)
        {
            error = "bind() failed WSA=" + std::to_string(WSAGetLastError())
                + " bind=" + SockaddrToString(bindAddr);
            Close();
            return false;
        }

        std::cout << "[NET][SOCK] Bound to " << SockaddrToString(bindAddr) << "\n";

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

    bool UdpSocket::Send(const sockaddr_storage& addr, int /*addrLen*/, const void* data, int size)
    {
        const int realLen = SockaddrLen(addr);

        const int sent = sendto(
            m_socket,
            static_cast<const char*>(data),
            size,
            0,
            reinterpret_cast<const sockaddr*>(&addr),
            realLen);

        if (sent != size)
        {
            std::cout << "[NET][SEND-ERR] to=" << SockaddrToString(addr)
                << " wanted=" << size << " sent=" << sent
                << " WSA=" << WSAGetLastError() << "\n";
            return false;
        }

        std::cout << "[NET][SEND] to=" << SockaddrToString(addr)
            << " bytes=" << sent << "\n";
        return true;
    }

    int UdpSocket::Receive(sockaddr_storage& from, void* buffer, int maxSize)
    {
        int fromLen = sizeof(from);

        const int result = recvfrom(
            m_socket,
            static_cast<char*>(buffer),
            maxSize,
            0,
            reinterpret_cast<sockaddr*>(&from),
            &fromLen);

        if (result == SOCKET_ERROR)
        {
            const int err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK)
                return 0;

            std::cout << "[NET][RECV-ERR] WSA=" << err << "\n";
            return -1;
        }

        std::cout << "[NET][RECV] from=" << SockaddrToString(from)
            << " bytes=" << result << "\n";

        return result;
    }
}