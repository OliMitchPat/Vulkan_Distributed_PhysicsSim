#include "PeerConfig.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>

namespace Net {

// ---- Internal helpers -------------------------------------------------------

static std::string TrimWhitespace(const std::string& s)
{
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
        return {};
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

// Parse a single "peerId@host:port" token.
// Returns true on success.
static bool ParsePeerEntry(const std::string& token,
                            PeerEntry&         out,
                            std::string&       errorOut)
{
    // Find '@'
    const auto atPos = token.find('@');
    if (atPos == std::string::npos)
    {
        errorOut = "peer entry missing '@': " + token;
        return false;
    }

    const std::string idStr = TrimWhitespace(token.substr(0, atPos));
    const std::string rest  = TrimWhitespace(token.substr(atPos + 1));

    // Parse peerId
    try
    {
        const int id = std::stoi(idStr);
        if (id < 1 || id > 4)
        {
            errorOut = "peer id out of range (1..4): " + idStr;
            return false;
        }
        out.peerId = id;
    }
    catch (...)
    {
        errorOut = "invalid peer id: " + idStr;
        return false;
    }

    // Find last ':' to split host:port (supports IPv6 literals without brackets)
    const auto colonPos = rest.rfind(':');
    if (colonPos == std::string::npos)
    {
        errorOut = "peer entry missing port after host: " + rest;
        return false;
    }

    out.host = TrimWhitespace(rest.substr(0, colonPos));
    const std::string portStr = TrimWhitespace(rest.substr(colonPos + 1));

    if (out.host.empty())
    {
        errorOut = "peer entry has empty host";
        return false;
    }

    try
    {
        const int p = std::stoi(portStr);
        if (p < 1 || p > 65535)
        {
            errorOut = "peer port out of range (1..65535): " + portStr;
            return false;
        }
        out.port = static_cast<uint16_t>(p);
    }
    catch (...)
    {
        errorOut = "invalid peer port: " + portStr;
        return false;
    }

    return true;
}

// Parse the value of the "peers" key.
static bool ParsePeersList(const std::string&       value,
                            std::vector<PeerEntry>&  out,
                            std::string&             errorOut)
{
    std::stringstream ss(value);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        token = TrimWhitespace(token);
        if (token.empty())
            continue;

        PeerEntry entry;
        if (!ParsePeerEntry(token, entry, errorOut))
            return false;
        out.push_back(entry);
    }
    return true;
}

// ---- Public API -------------------------------------------------------------

std::vector<PeerEntry> PeerConfig::RemotePeers() const
{
    std::vector<PeerEntry> remotes;
    for (const auto& p : peers)
    {
        if (p.peerId != peer_id)
            remotes.push_back(p);
    }
    return remotes;
}

bool ParsePeerConfig(const std::string& path,
                     PeerConfig&        cfg,
                     std::string&       errorOut)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        errorOut = "cannot open config file: " + path;
        return false;
    }

    cfg = PeerConfig{};           // reset to defaults
    bool hasPeerId    = false;
    bool hasBindPort  = false;
    int  lineNumber   = 0;

    std::string line;
    while (std::getline(file, line))
    {
        ++lineNumber;
        line = TrimWhitespace(line);

        // Skip blank lines and comments
        if (line.empty() || line[0] == '#' || line[0] == ';')
            continue;

        // Split on first '='
        const auto eqPos = line.find('=');
        if (eqPos == std::string::npos)
        {
            errorOut = "line " + std::to_string(lineNumber) +
                       ": expected 'key = value', got: " + line;
            return false;
        }

        const std::string key   = TrimWhitespace(line.substr(0, eqPos));
        const std::string value = TrimWhitespace(line.substr(eqPos + 1));

        if (key == "peer_id")
        {
            try
            {
                const int id = std::stoi(value);
                if (id < 1 || id > 4)
                {
                    errorOut = "peer_id out of range (1..4): " + value;
                    return false;
                }
                cfg.peer_id = id;
                hasPeerId = true;
            }
            catch (...)
            {
                errorOut = "invalid peer_id: " + value;
                return false;
            }
        }
        else if (key == "bind_ip")
        {
            cfg.bind_ip = value;
        }
        else if (key == "bind_port")
        {
            try
            {
                const int p = std::stoi(value);
                if (p < 1 || p > 65535)
                {
                    errorOut = "bind_port out of range (1..65535): " + value;
                    return false;
                }
                cfg.bind_port = static_cast<uint16_t>(p);
                hasBindPort = true;
            }
            catch (...)
            {
                errorOut = "invalid bind_port: " + value;
                return false;
            }
        }
        else if (key == "peers")
        {
            cfg.peers.clear();
            if (!ParsePeersList(value, cfg.peers, errorOut))
                return false;
        }
        else if (key == "render_hz")
        {
            try
            {
                const float v = std::stof(value);
                if (v <= 0.0f) { errorOut = "render_hz must be > 0: " + value; return false; }
                cfg.render_hz = v;
            }
            catch (...) { errorOut = "invalid render_hz: "     + value; return false; }
        }
        else if (key == "network_hz")
        {
            try
            {
                const float v = std::stof(value);
                if (v <= 0.0f) { errorOut = "network_hz must be > 0: " + value; return false; }
                cfg.network_hz = v;
            }
            catch (...) { errorOut = "invalid network_hz: "    + value; return false; }
        }
        else if (key == "simulation_hz")
        {
            try
            {
                const float v = std::stof(value);
                if (v <= 0.0f) { errorOut = "simulation_hz must be > 0: " + value; return false; }
                cfg.simulation_hz = v;
            }
            catch (...) { errorOut = "invalid simulation_hz: " + value; return false; }
        }
        // Unknown keys are silently ignored (forward-compatible)
    }

    if (!hasPeerId)
    {
        errorOut = "missing required field: peer_id";
        return false;
    }
    if (!hasBindPort)
    {
        errorOut = "missing required field: bind_port";
        return false;
    }

    return true;
}

} // namespace Net
