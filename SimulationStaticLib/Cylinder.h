#pragma once
#include <glm/glm.hpp>

struct Cylinder
{
    float radius = 0.5f;
    float height = 1.0f; // full height along local Y

    Cylinder() = default;
    Cylinder(float r, float h) : radius(r), height(h) {}

    float halfHeight() const { return 0.5f * height; }
};