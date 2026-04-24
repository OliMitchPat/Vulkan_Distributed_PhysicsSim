#pragma once
/*
 * PeerConfig.h — INI-style configuration parser for the peer-to-peer simulator.
 *
 * File format (INI-like, UTF-8):
 *   - Lines beginning with '#' or ';' are comments and are ignored.
 *   - Blank / whitespace-only lines are ignored.
 *   - Key-value pairs use the form:   key = value
 *     Whitespace around the '=' and at the end of the line is trimmed.
 *
 * Recognised keys:
 *   peer_id       Integer 1..4 — identity of this node.
 *   bind_ip       IPv4/IPv6 address or hostname to bind (default: "0.0.0.0").
 *   bind_port     UDP port (1..65535) to listen on.
 *   peers         Comma-separated list in the form
 *                   <peerId>@<host>:<port> [, <peerId>@<host>:<port> ...]
 *                 e.g.  peers = 1@192.168.1.10:9001, 2@192.168.1.11:9002
 *                 Self may be included; use RemotePeers() to skip self.
 *   render_hz     Target graphics loop frequency (optional, default 60).
 *   network_hz    Target networking loop frequency (optional, default 60).
 *   simulation_hz Target physics loop frequency   (optional, default 120).
 */

#include <cstdint>
#include <string>
#include <vector>

namespace Net {

// A single entry from the 'peers' list.
struct PeerEntry
{
    int          peerId  = 0;
    std::string  host;          // hostname or IP address string
    uint16_t     port    = 0;
};

// Complete configuration for one simulation node.
struct PeerConfig
{
    int                    peer_id       = 0;
    std::string            bind_ip       = "0.0.0.0";
    uint16_t               bind_port     = 0;
    std::vector<PeerEntry> peers;

    // Optional loop-frequency hints (Hz).
    float render_hz     = 60.0f;
    float network_hz    = 60.0f;
    float simulation_hz = 120.0f;

    // Returns entries whose peerId != this node's peer_id.
    std::vector<PeerEntry> RemotePeers() const;
};

/*
 * Parse an INI config file at 'path'.
 * Returns true on success and populates 'cfg'.
 * On error, writes a description to 'errorOut' and returns false.
 */
bool ParsePeerConfig(const std::string& path,
                     PeerConfig&        cfg,
                     std::string&       errorOut);

} // namespace Net
