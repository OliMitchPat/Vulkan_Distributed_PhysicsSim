#include "ObjLoader.h"

#include <fstream>
#include <sstream>
#include <unordered_map>
#include <cctype>
#include <cmath>

static bool starts_with(const std::string& s, const char* prefix)
{
    for (size_t i = 0; prefix[i]; ++i)
        if (i >= s.size() || s[i] != prefix[i]) return false;
    return true;
}

static void trim_left(std::string& s)
{
    size_t i = 0;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    s.erase(0, i);
}

struct IdxTriplet
{
    int v = 0;
    int vt = 0;
    int vn = 0;

    bool operator==(const IdxTriplet& o) const;
};

bool IdxTriplet::operator==(const IdxTriplet& o) const
{
    return v == o.v && vt == o.vt && vn == o.vn;
}

struct IdxTripletHash
{
    size_t operator()(const IdxTriplet& k) const noexcept;
};

size_t IdxTripletHash::operator()(const IdxTriplet& k) const noexcept
{
    // simple hash combine
    size_t const h1 = std::hash<int>{}(k.v);
    size_t const h2 = std::hash<int>{}(k.vt);
    size_t const h3 = std::hash<int>{}(k.vn);
    return (h1 * 1315423911u) ^ (h2 * 2654435761u) ^ (h3 * 97531u);
}


// Convert OBJ index (1-based, possibly negative) to 0-based.
// Returns -1 if index is 0 (missing).
static int resolve_index(int idx, int count)
{
    if (idx == 0) return -1;
    if (idx > 0) return idx - 1;
    // negative: -1 means last
    return count + idx;
}

// Parse one face token: "v", "v/vt", "v//vn", "v/vt/vn"
static bool parse_face_token(const std::string& tok, IdxTriplet& out)
{
    out = {};

    // We'll parse up to 3 ints separated by '/'
    // Examples:
    // "3" -> v=3
    // "3/2" -> v=3 vt=2
    // "3//7" -> v=3 vn=7
    // "3/2/7" -> v=3 vt=2 vn=7

    int parts[3] = { 0, 0, 0 };
    int partIndex = 0;
    std::string num;
    bool sawSlash = false;

    for (size_t i = 0; i <= tok.size(); ++i)
    {
        char const c = (i < tok.size()) ? tok[i] : '\0';

        if (c == '/' || c == '\0')
        {
            sawSlash = true;

            if (!num.empty())
            {
                parts[partIndex] = std::stoi(num);
                num.clear();
            }
            else
            {
                // empty means 0 (missing)
                parts[partIndex] = 0;
            }

            ++partIndex;
            if (partIndex > 2) break;
            continue;
        }

        num.push_back(c);
    }

    if (!sawSlash)
    {
        // Just "v"
        out.v = std::stoi(tok);
        out.vt = 0;
        out.vn = 0;
        return true;
    }

    // If token had slashes:
    // partIndex is number of separators+1, but we capped at 3.
    // Map:
    out.v = parts[0];
    out.vt = parts[1];
    out.vn = parts[2];
    return true;
}

bool LoadObj(const std::string& path, ObjMeshData& outMesh, std::string* outError)
{
    outMesh = {};

    std::ifstream file(path);
    if (!file.is_open())
    {
        if (outError) *outError = "LoadObj: failed to open file: " + path;
        return false;
    }

    std::vector<glm::vec3> positions;
    std::vector<glm::vec2> texcoords;
    std::vector<glm::vec3> normals;

    // Dedup key: (v,vt,vn) -> output vertex index
    std::unordered_map<IdxTriplet, uint32_t, IdxTripletHash> dedup;

    std::string line;
    int lineNo = 0;

    while (std::getline(file, line))
    {
        ++lineNo;
        trim_left(line);
        if (line.empty() || line[0] == '#') continue;

        if (starts_with(line, "v "))
        {
            std::istringstream iss(line);
            char vchar;
            float x, y, z;
            iss >> vchar >> x >> y >> z;
            positions.push_back({ x, y, z });
        }
        else if (starts_with(line, "vt "))
        {
            std::istringstream iss(line);
            std::string tag;
            float u, v;
            iss >> tag >> u >> v;
            texcoords.push_back({ u, v });
        }
        else if (starts_with(line, "vn "))
        {
            std::istringstream iss(line);
            std::string tag;
            float x, y, z;
            iss >> tag >> x >> y >> z;
            normals.push_back({ x, y, z });
        }
        else if (starts_with(line, "f "))
        {
            std::istringstream iss(line);
            char fchar;
            iss >> fchar;

            // Read all face tokens
            std::vector<IdxTriplet> face;
            std::string tok;
            while (iss >> tok)
            {
                IdxTriplet t{};
                if (!parse_face_token(tok, t))
                {
                    if (outError) *outError = "LoadObj: bad face token at line " + std::to_string(lineNo);
                    return false;
                }
                face.push_back(t);
            }

            if (face.size() < 3) continue;

            // Triangulate fan: (0, i, i+1)
            for (size_t i = 1; i + 1 < face.size(); ++i)
            {
                IdxTriplet tri[3] = { face[0], face[i], face[i + 1] };

                for (int k = 0; k < 3; ++k)
                {
                    IdxTriplet const key = tri[k];

                    int const pv = resolve_index(key.v, static_cast<int>(positions.size()));
                    int const pt = resolve_index(key.vt, static_cast<int>(texcoords.size()));
                    int const pn = resolve_index(key.vn, static_cast<int>(normals.size()));

                    if (pv < 0 || pv >= static_cast<int>(positions.size()))
                    {
                        if (outError) *outError = "LoadObj: position index out of range at line " + std::to_string(lineNo);
                        return false;
                    }

                    // Normalize key to resolved indices for dedup consistency
                    IdxTriplet resolvedKey{};
                    resolvedKey.v = pv;
                    resolvedKey.vt = pt;
                    resolvedKey.vn = pn;

                    auto const it = dedup.find(resolvedKey);
                    if (it != dedup.end())
                    {
                        outMesh.indices.push_back(it->second);
                        continue;
                    }

                    ObjVertex ov{};
                    ov.pos = positions[pv];

                    if (pt >= 0 && pt < static_cast<int>(texcoords.size()))
                    {
                        // OBJ uv origin is typically bottom-left; Vulkan textures often expect top-left.
                        // We'll flip V so textures look correct (common practice).
                        glm::vec2 const uv = texcoords[pt];
                        ov.uv = { uv.x, 1.0f - uv.y };
                    }

                    if (pn >= 0 && pn < static_cast<int>(normals.size()))
                    {
                        ov.normal = normals[pn];
                    }

                    uint32_t const newIndex = static_cast<uint32_t>(outMesh.vertices.size());
                    outMesh.vertices.push_back(ov);
                    outMesh.indices.push_back(newIndex);
                    dedup[resolvedKey] = newIndex;
                }
            }
        }
        else
        {
            // Ignore other statements for coursework scope
            continue;
        }
    }

    if (outMesh.vertices.empty() || outMesh.indices.empty())
    {
        if (outError) *outError = "LoadObj: mesh contains no triangles: " + path;
        return false;
    }

    return true;
}
