#pragma once
#include <glm/glm.hpp>

// World-space sphere (center + radius)
struct WorldSphere
{
    glm::vec3 center{ 0.0f };
    float radius = 0.5f;
};

// World-space plane (point on plane + unit normal)
struct WorldPlane
{
    glm::vec3 point{ 0.0f };
    glm::vec3 normal{ 0.0f, 1.0f, 0.0f }; // must be normalized
};

// World-space capsule represented as a segment AB with radius
struct WorldCapsule
{
    glm::vec3 a{ 0.0f };
    glm::vec3 b{ 0.0f };
    float radius = 0.5f;
};

// World-space finite cylinder represented as a segment AB (axis) with radius
struct WorldCylinder
{
    glm::vec3 a{ 0.0f };
    glm::vec3 b{ 0.0f };
    float radius = 0.5f;
};

// World-space oriented bounding box (OBB)
struct WorldOBB
{
    glm::vec3 center{ 0.0f };

    // Orthonormal basis axes (unit vectors)
    glm::vec3 axisX{ 1.0f, 0.0f, 0.0f };
    glm::vec3 axisY{ 0.0f, 1.0f, 0.0f };
    glm::vec3 axisZ{ 0.0f, 0.0f, 1.0f };

    // Half sizes along each axis (in world units)
    glm::vec3 halfExtents{ 0.5f, 0.5f, 0.5f };
};
