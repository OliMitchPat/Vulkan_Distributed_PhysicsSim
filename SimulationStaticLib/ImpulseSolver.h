#pragma once

#include "RigidBody.h"
#include "CollisionManifold.h"

#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>

struct ContactMaterial
{
    float restitution = 0.2f;
    float staticFriction = 0.5f;
    float dynamicFriction = 0.3f;
};

// Resting contact threshold (m/s). Below this, restitution is disabled to avoid jitter.
static constexpr float kRestitutionVelocityThreshold = 0.5f;

inline glm::vec3 VelocityAtPoint(const RigidBody& b, const glm::vec3& worldPoint)
{
    const glm::vec3 r = worldPoint - b.Position();
    return b.LinearVelocity() + glm::cross(b.AngularVelocity(), r);
}

inline void SolveContactImpulse(
    RigidBody& A,
    RigidBody& B,
    const CollisionManifold& m,
    const ContactMaterial& mat)
{
    if (!m.hit) return;

    if (!A.IsDynamic() && !B.IsDynamic()) return;

    const float invMassA = A.EffectiveInverseMass();
    const float invMassB = B.EffectiveInverseMass();
    const glm::mat3 invIA = A.EffectiveInverseInertiaWorld();
    const glm::mat3 invIB = B.EffectiveInverseInertiaWorld();

    const glm::vec3 n = m.normal;
    const glm::vec3 p = m.contactPoint;

    const glm::vec3 ra = p - A.Position();
    const glm::vec3 rb = p - B.Position();

    const glm::vec3 va = VelocityAtPoint(A, p);
    const glm::vec3 vb = VelocityAtPoint(B, p);
    const glm::vec3 rv = vb - va;

    const float velAlongNormal = glm::dot(rv, n);
    if (velAlongNormal > 0.0f) return;

    float e = std::clamp(mat.restitution, 0.0f, 1.0f);
    if (std::abs(velAlongNormal) < kRestitutionVelocityThreshold)
        e = 0.0f;

    // ---- Normal impulse ----
    const glm::vec3 raXn = glm::cross(ra, n);
    const glm::vec3 rbXn = glm::cross(rb, n);

    const float angA = glm::dot(glm::cross(invIA * raXn, ra), n);
    const float angB = glm::dot(glm::cross(invIB * rbXn, rb), n);

    const float denom = invMassA + invMassB + angA + angB;
    if (std::abs(denom) < 1e-8f) return;

    const float j = -(1.0f + e) * velAlongNormal / denom;
    const glm::vec3 impulse = j * n;

    A.ApplyImpulseAtPoint(-impulse, p);
    B.ApplyImpulseAtPoint(impulse, p);

    // ---- Friction impulse ----
    const glm::vec3 va2 = VelocityAtPoint(A, p);
    const glm::vec3 vb2 = VelocityAtPoint(B, p);
    const glm::vec3 rv2 = vb2 - va2;

    glm::vec3 t = rv2 - glm::dot(rv2, n) * n;
    const float tLen2 = glm::dot(t, t);
    if (tLen2 < 1e-10f) return;

    t *= 1.0f / std::sqrt(tLen2);

    const glm::vec3 raXt = glm::cross(ra, t);
    const glm::vec3 rbXt = glm::cross(rb, t);

    const float angAt = glm::dot(glm::cross(invIA * raXt, ra), t);
    const float angBt = glm::dot(glm::cross(invIB * rbXt, rb), t);

    const float denomT = invMassA + invMassB + angAt + angBt;
    if (std::abs(denomT) < 1e-8f) return;

    float jt = -glm::dot(rv2, t) / denomT;

    const float muS = std::max(0.0f, mat.staticFriction);
    const float muD = std::max(0.0f, mat.dynamicFriction);

    glm::vec3 frictionImpulse;
    if (std::abs(jt) < j * muS)
        frictionImpulse = jt * t;
    else
        frictionImpulse = (-j * muD) * t;

    A.ApplyImpulseAtPoint(-frictionImpulse, p);
    B.ApplyImpulseAtPoint(frictionImpulse, p);
}