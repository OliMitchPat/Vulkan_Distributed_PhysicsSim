// name=CppGoogleTest/Response-ImpulseSolver.cpp
#include "pch.h"
#include <gtest/gtest.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "RigidBody.h"
#include "ImpulseSolver.h"
#include "CollisionManifold.h"
#include "TestUtils.h"

static RigidBody MakeDynamicBody(
    const glm::vec3& pos,
    float mass,
    const glm::mat3& invInertiaBody = glm::mat3(0.0f)) // default: no rotation response
{
    RigidBody b;
    b.SetMotionType(BodyMotionType::Dynamic);
    b.SetPosition(pos);
    b.SetOrientation(glm::quat(1, 0, 0, 0));
    b.SetLinearVelocity(glm::vec3(0));
    b.SetAngularVelocity(glm::vec3(0));
    b.SetMass(mass);
    b.SetInverseInertiaBody(invInertiaBody);
    return b;
}

TEST(Response_ImpulseSolver, NoChange_WhenNotHit)
{
    RigidBody A = MakeDynamicBody(glm::vec3(0, 0, 0), 1.0f);
    RigidBody B = MakeDynamicBody(glm::vec3(1, 0, 0), 1.0f);

    A.SetLinearVelocity(glm::vec3(1, 0, 0));
    B.SetLinearVelocity(glm::vec3(-1, 0, 0));

    CollisionManifold m{};
    m.hit = false;

    ContactMaterial mat{};
    SolveContactImpulse(A, B, m, mat);

    ExpectVec3Near(A.LinearVelocity(), glm::vec3(1, 0, 0), 1e-6f);
    ExpectVec3Near(B.LinearVelocity(), glm::vec3(-1, 0, 0), 1e-6f);
}

TEST(Response_ImpulseSolver, NoChange_WhenSeparatingAlongNormal)
{
    // Normal A->B = +X, but bodies moving apart => velAlongNormal > 0 => early return
    RigidBody A = MakeDynamicBody(glm::vec3(0, 0, 0), 1.0f);
    RigidBody B = MakeDynamicBody(glm::vec3(1, 0, 0), 1.0f);

    A.SetLinearVelocity(glm::vec3(-1, 0, 0));
    B.SetLinearVelocity(glm::vec3(1, 0, 0));

    CollisionManifold m{};
    m.hit = true;
    m.normal = glm::vec3(1, 0, 0);
    m.contactPoint = glm::vec3(0.5f, 0, 0);

    ContactMaterial mat{};
    mat.restitution = 1.0f;

    SolveContactImpulse(A, B, m, mat);

    ExpectVec3Near(A.LinearVelocity(), glm::vec3(-1, 0, 0), 1e-6f);
    ExpectVec3Near(B.LinearVelocity(), glm::vec3(1, 0, 0), 1e-6f);
}

TEST(Response_ImpulseSolver, PerfectlyElastic_EqualMass_HeadOn_SwapsVelocities)
{
    // Head-on along +X, equal mass, e=1 => swap velocities
    RigidBody A = MakeDynamicBody(glm::vec3(0, 0, 0), 1.0f);
    RigidBody B = MakeDynamicBody(glm::vec3(1, 0, 0), 1.0f);

    A.SetLinearVelocity(glm::vec3(1, 0, 0));
    B.SetLinearVelocity(glm::vec3(-1, 0, 0));

    CollisionManifold m{};
    m.hit = true;
    m.normal = glm::vec3(1, 0, 0);          // A -> B
    m.contactPoint = glm::vec3(0.5f, 0, 0); // on line between them

    ContactMaterial mat{};
    mat.restitution = 1.0f;
    mat.staticFriction = 0.0f;
    mat.dynamicFriction = 0.0f;

    SolveContactImpulse(A, B, m, mat);

    ExpectVec3Near(A.LinearVelocity(), glm::vec3(-1, 0, 0), 1e-4f);
    ExpectVec3Near(B.LinearVelocity(), glm::vec3(1, 0, 0), 1e-4f);
}

TEST(Response_ImpulseSolver, PerfectlyInelastic_EqualMass_HeadOn_BothStop)
{
    // e=0, equal mass, opposite velocities => both should go to 0
    RigidBody A = MakeDynamicBody(glm::vec3(0, 0, 0), 1.0f);
    RigidBody B = MakeDynamicBody(glm::vec3(1, 0, 0), 1.0f);

    A.SetLinearVelocity(glm::vec3(1, 0, 0));
    B.SetLinearVelocity(glm::vec3(-1, 0, 0));

    CollisionManifold m{};
    m.hit = true;
    m.normal = glm::vec3(1, 0, 0);
    m.contactPoint = glm::vec3(0.5f, 0, 0);

    ContactMaterial mat{};
    mat.restitution = 0.0f;
    mat.staticFriction = 0.0f;
    mat.dynamicFriction = 0.0f;

    SolveContactImpulse(A, B, m, mat);

    ExpectVec3Near(A.LinearVelocity(), glm::vec3(0, 0, 0), 1e-4f);
    ExpectVec3Near(B.LinearVelocity(), glm::vec3(0, 0, 0), 1e-4f);
}

TEST(Response_ImpulseSolver, StaticBody_OnlyDynamicChanges)
{
    RigidBody A = MakeDynamicBody(glm::vec3(0, 0, 0), 1.0f);
    RigidBody B; // static
    B.SetMotionType(BodyMotionType::Static);
    B.SetPosition(glm::vec3(1, 0, 0));

    A.SetLinearVelocity(glm::vec3(1, 0, 0));
    B.SetLinearVelocity(glm::vec3(0, 0, 0)); // should be treated as zero anyway

    CollisionManifold m{};
    m.hit = true;
    m.normal = glm::vec3(1, 0, 0);
    m.contactPoint = glm::vec3(0.5f, 0, 0);

    ContactMaterial mat{};
    mat.restitution = 1.0f;
    mat.staticFriction = 0.0f;
    mat.dynamicFriction = 0.0f;

    SolveContactImpulse(A, B, m, mat);

    // Dynamic should bounce back; static unchanged
    EXPECT_LT(A.LinearVelocity().x, 0.0f);
    ExpectVec3Near(B.LinearVelocity(), glm::vec3(0, 0, 0), 1e-6f);
}

TEST(Response_ImpulseSolver, Friction_ReducesTangentialSpeed)
{
    // Normal +Y. Make B moving downward into A (closing along -Y),
    // and also sliding along +X (tangential).
    RigidBody A = MakeDynamicBody(glm::vec3(0, 0, 0), 1.0f);
    RigidBody B = MakeDynamicBody(glm::vec3(0, 0, 0), 1.0f);

    A.SetLinearVelocity(glm::vec3(0, 0, 0));
    B.SetLinearVelocity(glm::vec3(10.0f, -1.0f, 0.0f)); // tangential +X, closing -Y

    CollisionManifold m{};
    m.hit = true;
    m.normal = glm::vec3(0, 1, 0);
    m.contactPoint = glm::vec3(0, 0, 0);

    ContactMaterial mat{};
    mat.restitution = 0.0f;
    mat.staticFriction = 1.0f;
    mat.dynamicFriction = 1.0f;

    SolveContactImpulse(A, B, m, mat);

    // Tangential should reduce in magnitude:
    EXPECT_LT(B.LinearVelocity().x, 10.0f);

    // And they should not be closing along the normal anymore (or at least less so):
    EXPECT_GE(B.LinearVelocity().y, -1.0f);
}