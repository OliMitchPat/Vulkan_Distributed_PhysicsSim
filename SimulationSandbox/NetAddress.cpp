#include "NetAddress.h"

#pragma comment(lib, "Ws2_32.lib")

#include <cstring>
#include <stdexcept>

namespace Net {

bool ResolveAddress(const std::string& host,
                    uint16_t           port,
                    sockaddr_storage&  addrOut,
                    std::string&       errorOut)
{
    addrinfo hints{};
    hints.ai_family   = AF_UNSPEC;      // accept IPv4 or IPv6
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags    = AI_NUMERICSERV; // port is numeric

    const std::string portStr = std::to_string(port);

    addrinfo* results = nullptr;
    const int rc = getaddrinfo(host.c_str(), portStr.c_str(), &hints, &results);
    if (rc != 0)
    {
        errorOut = "getaddrinfo failed for \"" + host + ":" + portStr +
                   "\": " + gai_strerrorA(rc);
        return false;
    }

    // Prefer IPv4; fall back to IPv6
    addrinfo* chosen = nullptr;
    for (addrinfo* ai = results; ai != nullptr; ai = ai->ai_next)
    {
        if (ai->ai_family == AF_INET)
        {
            chosen = ai;
            break;
        }
        if (!chosen && ai->ai_family == AF_INET6)
            chosen = ai;
    }

    if (!chosen)
    {
        freeaddrinfo(results);
        errorOut = "no usable address found for \"" + host + "\"";
        return false;
    }

    std::memset(&addrOut, 0, sizeof(addrOut));
    std::memcpy(&addrOut, chosen->ai_addr,
                static_cast<size_t>(chosen->ai_addrlen));

    freeaddrinfo(results);
    return true;
}

std::string AddressToString(const sockaddr_storage& addr)
{
    char ipBuf[INET6_ADDRSTRLEN] = {};

    if (addr.ss_family == AF_INET)
    {
        const auto* s4 = reinterpret_cast<const sockaddr_in*>(&addr);
        inet_ntop(AF_INET, &s4->sin_addr, ipBuf, sizeof(ipBuf));
        const uint16_t port = ntohs(s4->sin_port);
        return std::string(ipBuf) + ":" + std::to_string(port);
    }
    else if (addr.ss_family == AF_INET6)
    {
        const auto* s6 = reinterpret_cast<const sockaddr_in6*>(&addr);
        inet_ntop(AF_INET6, &s6->sin6_addr, ipBuf, sizeof(ipBuf));
        const uint16_t port = ntohs(s6->sin6_port);
        return "[" + std::string(ipBuf) + "]:" + std::to_string(port);
    }

    return "<unknown>";
}

} // namespace Net
