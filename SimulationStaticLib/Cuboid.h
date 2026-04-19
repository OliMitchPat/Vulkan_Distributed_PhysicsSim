#pragma once
#include <glm/glm.hpp>

struct Cuboid
{
    // Full size (width, height, depth) in local space
    glm::vec3 size{ 1.0f, 1.0f, 1.0f };

    Cuboid() = default;
    explicit Cuboid(const glm::vec3& s) : size(s) {}

    glm::vec3 halfExtents() const { return 0.5f * size; }
};