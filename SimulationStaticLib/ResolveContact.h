#pragma once
#include "CollisionManifold.h"
#include "ImpulseSolver.h"
#include "PositionalCorrection.h"

inline void ResolveContactVelocity(
    RigidBody& A,
    RigidBody& B,
    const CollisionManifold& m,
    const ContactMaterial& mat)
{
    if (!m.hit) return;

    // Handles bounce, stopping, friction, and angular response.
    SolveContactImpulse(A, B, m, mat);
}

inline void ResolveContactPosition(
    RigidBody& A,
    RigidBody& B,
    const CollisionManifold& m)
{
    if (!m.hit) return;

    // Moves objects out of penetration.
    // This should usually be done once per frame, not once per solver iteration.
    PositionalCorrection(A, B, m);
}

inline void ResolveContact(
    RigidBody& A,
    RigidBody& B,
    const CollisionManifold& m,
    const ContactMaterial& mat)
{
    if (!m.hit) return;

    ResolveContactVelocity(A, B, m, mat);
    ResolveContactPosition(A, B, m);
}