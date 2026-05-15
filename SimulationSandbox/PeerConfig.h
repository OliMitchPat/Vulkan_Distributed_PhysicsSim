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
 *   control_bind_port   UDP control port (1..65535) to listen on.
 *   snapshot_bind_port  UDP snapshot port (1..65535) to listen on.
 *   bind_port           Legacy single UDP port; maps to:
 *                       control_bind_port = bind_port
 *                       snapshot_bind_port = bind_port + 1
 *
 *   peers         Legacy comma-separated list in the form
 *                   <peerId>@<host>:<port> [, <peerId>@<host>:<port> ...]
 *                 where <port> maps to control=<port>, snapshot=<port+1>.
 *
 *   peerN_host / peerN_control_port / peerN_snapshot_port
 *                 Structured peer entries for N=1..4.
 *   render_hz     Target graphics loop frequency (optional, default 60).
 *   network_hz    Target networking loop frequency (optional, default 60).
 *   simulation_hz Target physics loop frequency   (optional, default 120).
 */

#include <cstdint>
#include <string>
#include <vector>

namespace Net {

// A single remote-peer entry.
struct RemotePeerConfig
{
    int          peerId  = 0;
    std::string  host;          // hostname or IP address string
    uint16_t     control_port = 0;
    uint16_t     snapshot_port = 0;
};

// Complete configuration for one simulation node.
struct PeerConfig
{
    int                    peer_id       = 0;
    std::string            bind_ip       = "0.0.0.0";
    uint16_t               control_bind_port = 0;
    uint16_t               snapshot_bind_port = 0;
    std::vector<RemotePeerConfig> peers;

    // Optional loop-frequency hints (Hz).
    float render_hz     = 60.0f;
    float network_hz    = 60.0f;
    float simulation_hz = 120.0f;

    // Returns entries whose peerId != this node's peer_id.
    std::vector<RemotePeerConfig> RemotePeers() const;
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
