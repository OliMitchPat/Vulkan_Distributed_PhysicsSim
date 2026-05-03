#pragma once

#include "CollisionManifold.h"
#include "RigidBody.h"

#include <algorithm>
#include <glm/glm.hpp>

inline void PositionalCorrection(
    RigidBody& A,
    RigidBody& B,
    const CollisionManifold& m,
    float percent = 0.3f,   // 80% of penetration
    float slop = 0.01f)    // small allowance
{
    if (!m.hit) return;

    const float invMassA = A.InverseMass();
    const float invMassB = B.InverseMass();
    const float invMassSum = invMassA + invMassB;
    if (invMassSum == 0.0f) return;

    const float correctionMag =
        std::max(m.penetration - slop, 0.0f) / invMassSum * percent;

    const glm::vec3 correction = correctionMag * m.normal;

    A.SetPosition(A.Position() - correction * invMassA);
    B.SetPosition(B.Position() + correction * invMassB);
}