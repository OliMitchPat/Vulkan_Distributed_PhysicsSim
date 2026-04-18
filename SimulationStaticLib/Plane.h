#pragma once
#include <glm/glm.hpp>

struct PlaneShape
{
    // Local-space normal; should be normalized when used
    glm::vec3 normal{ 0.0f, 1.0f, 0.0f };

    PlaneShape() = default;
    explicit PlaneShape(const glm::vec3& n) : normal(n) {}
};