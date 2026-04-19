#pragma once
#include <glm/glm.hpp>

// Basic collision manifold (single-contact).
// Normal convention: points from shape A toward shape B.
struct CollisionManifold
{
    bool hit = false;

    // Unit normal from A -> B (if hit==true)
    glm::vec3 normal{ 1.0f, 0.0f, 0.0f };

    // Penetration depth (>= 0). 0 means touching.
    float penetration = 0.0f;

    // One contact point in world space (best-effort).
    // For many pairs this will be the point on the surface of A (or midpoint).
    glm::vec3 contactPoint{ 0.0f };

    static CollisionManifold NoHit() { return {}; }
};