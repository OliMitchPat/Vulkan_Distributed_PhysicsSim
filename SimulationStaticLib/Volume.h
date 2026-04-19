#pragma once
#include <glm/glm.hpp>
#include <numbers>

inline float Volume_Sphere(float r)
{
    return (4.0f / 3.0f) * std::numbers::pi_v<float> *r * r * r;
}

inline float Volume_Cuboid(const glm::vec3& size)
{
    return size.x * size.y * size.z;
}

inline float Volume_Cylinder(float r, float h)
{
    return std::numbers::pi_v<float> *r * r * h;
}

inline float Volume_Capsule(float r, float h)
{
    // cylinder + sphere
    return Volume_Cylinder(r, h) + Volume_Sphere(r);
}