// name=SimulationStaticLib/Collision/CollisionResponse.h
#pragma once
#include "CollisionManifold.h"
#include "PhysicsObject.h"
#include <algorithm>
#include <glm/glm.hpp>

// Resolve A vs B using a single contact manifold.
// Assumes manifold.normal points from A -> B.
inline void ResolveCollisionLinear(
    PhysicsObject& A,
    PhysicsObject& B,
    const CollisionManifold& m,
    float restitution)

{
    if (!m.hit) return;

    // If both static, nothing to do.
    const float invMassA = A.InverseMass();
    const float invMassB = B.InverseMass();
    if (invMassA == 0.0f && invMassB == 0.0f) return;

    const glm::vec3 n = m.normal;

    // Relative velocity (B relative to A)
    const glm::vec3 rv = B.Velocity() - A.Velocity();
    const float velAlongNormal = glm::dot(rv, n);

    // If separating, don't apply impulse
    if (velAlongNormal > 0.0f) return;

    // Clamp restitution
    const float e = std::clamp(restitution, 0.0f, 1.0f);

    // Impulse scalar
    const float j = -(1.0f + e) * velAlongNormal / (invMassA + invMassB);

    const glm::vec3 impulse = j * n;

    A.ApplyImpulse(-impulse);
    B.ApplyImpulse(impulse);
}