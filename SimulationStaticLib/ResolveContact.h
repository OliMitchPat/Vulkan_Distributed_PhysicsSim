#pragma once
#include "CollisionManifold.h"
#include "ImpulseSolver.h"
#include "PositionalCorrection.h"

inline void ResolveContact(
    RigidBody& A,
    RigidBody& B,
    const CollisionManifold& m,
    const ContactMaterial& mat)
{
    if (!m.hit) return;

    // position first so they are not deeply interpenetrating
    PositionalCorrection(A, B, m);

    // then velocity/rotation response
    SolveContactImpulse(A, B, m, mat);
}
