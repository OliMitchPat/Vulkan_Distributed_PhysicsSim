#pragma once
#include <glm/glm.hpp>

struct Capsule
{
    float radius = 0.5f;
    float height = 1.0f; // full height of the cylindrical section along local Y

    Capsule() = default;
    Capsule(float r, float h) : radius(r), height(h) {}

    float halfHeight() const { return 0.5f * height; }
};