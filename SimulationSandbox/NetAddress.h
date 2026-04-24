#pragma once
/*
 * NetAddress.h — Winsock2 hostname/IP resolution helpers.
 *
 * Uses getaddrinfo() to resolve a (host, port) pair to a sockaddr_storage.
 * IPv4 is preferred; IPv6 is accepted as a fallback.
 *
 * Winsock2 must be initialised (WSAStartup) before calling these functions.
 */

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <cstdint>
#include <string>

namespace Net {

/*
 * Resolve 'host' (hostname or numeric IP) and 'port' to a sockaddr_storage.
 *
 * Prefers AF_INET (IPv4); falls back to AF_INET6 if no IPv4 result exists.
 * Returns true on success and populates 'addrOut'.
 * On failure writes a description to 'errorOut' and returns false.
 */
bool ResolveAddress(const std::string& host,
                    uint16_t           port,
                    sockaddr_storage&  addrOut,
                    std::string&       errorOut);

/*
 * Returns a human-readable string for an address stored in 'addr',
 * e.g. "192.168.1.10:9001" or "[::1]:9001".
 * Returns "<unknown>" if the family is unsupported.
 */
std::string AddressToString(const sockaddr_storage& addr);

} // namespace Net
