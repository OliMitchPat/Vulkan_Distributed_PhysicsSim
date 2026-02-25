#pragma once
#include <string>
#include <vector>
#include <glm/glm.hpp>

struct ObjVertex
{
    glm::vec3 pos{ 0.0f };
    glm::vec3 normal{ 0.0f, 0.0f, 1.0f };
    glm::vec2 uv{ 0.0f };
};

struct ObjMeshData
{
    std::vector<ObjVertex> vertices;
    std::vector<uint32_t> indices;
};

bool LoadObj(const std::string& path, ObjMeshData& outMesh, std::string* outError = nullptr);
