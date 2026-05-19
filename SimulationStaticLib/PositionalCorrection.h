#pragma once

#include "CollisionManifold.h"
#include "RigidBody.h"

#include <algorithm>
#include <glm/glm.hpp>

inline void PositionalCorrection(
    RigidBody& A,
    RigidBody& B,
    const CollisionManifold& m,
    float percent = 0.8f,
    float slop = 0.001f)
{
    if (!m.hit) return;

    // Kinematic animated bodies are externally moved platforms/obstacles.
    // Leaving the usual loose correction slop here makes dynamic bodies appear
    // slightly embedded while resting on animated geometry, especially on peers
    // that receive the kinematic surface over the network.
    if ((A.IsDynamic() && B.IsKinematic()) || (A.IsKinematic() && B.IsDynamic()))
    {
        percent = 1.0f;
        slop = 0.00005f;
    }

    const float invMassA = A.EffectiveInverseMass();
    const float invMassB = B.EffectiveInverseMass();
    const float invMassSum = invMassA + invMassB;
    if (invMassSum == 0.0f) return;

    const float correctionMag =
        std::max(m.penetration - slop, 0.0f) / invMassSum * percent;
    if (correctionMag <= 0.0f) return;

    const glm::vec3 correction = correctionMag * m.normal;

    A.SetPosition(A.Position() - correction * invMassA);
    B.SetPosition(B.Position() + correction * invMassB);
}
