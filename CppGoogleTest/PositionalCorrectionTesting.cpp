#include "pch.h"
#include <gtest/gtest.h>

#include <glm/glm.hpp>

#include "RigidBody.h"
#include "CollisionManifold.h"
#include "PositionalCorrection.h"
#include "TestUtils.h"

static RigidBody MakeDynamicBody(
    const glm::vec3& pos,
    float mass,
    const glm::mat3& invInertiaBody = glm::mat3(0.0f))
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


TEST(Response_PositionalCorrection, BasicBehaviour)
{
    // ---- 1. No change when not hit ----
    {
        RigidBody A, B;
        A.SetPosition({ 0,0,0 });
        B.SetPosition({ 1,0,0 });

        CollisionManifold m{};
        m.hit = false;

        PositionalCorrection(A, B, m);

        ExpectVec3Near(A.Position(), glm::vec3(0, 0, 0), 1e-6f);
        ExpectVec3Near(B.Position(), glm::vec3(1, 0, 0), 1e-6f);
    }

    // ---- 2. Resolves penetration ----
    {
        RigidBody A = MakeDynamicBody({ 0,0,0 }, 1.0f);
        RigidBody B = MakeDynamicBody({ 0,0,0 }, 1.0f);

        CollisionManifold m{};
        m.hit = true;
        m.normal = glm::vec3(1, 0, 0);
        m.penetration = 1.0f;

        PositionalCorrection(A, B, m);

        EXPECT_LT(A.Position().x, 0.0f);
        EXPECT_GT(B.Position().x, 0.0f);
    }

    // ---- 3. Heavier body moves less ----
    {
        RigidBody A = MakeDynamicBody({ 0,0,0 }, 1.0f);
        RigidBody B = MakeDynamicBody({ 0,0,0 }, 10.0f);

        CollisionManifold m{};
        m.hit = true;
        m.normal = glm::vec3(1, 0, 0);
        m.penetration = 1.0f;

        PositionalCorrection(A, B, m);

        EXPECT_GT(std::abs(A.Position().x), std::abs(B.Position().x));
    }

    // ---- 4. Static body does not move ----
    {
        RigidBody A = MakeDynamicBody({ 0,0,0 }, 1.0f);
        RigidBody B;
        B.SetMotionType(BodyMotionType::Static);
        B.SetPosition({ 0,0,0 });

        CollisionManifold m{};
        m.hit = true;
        m.normal = glm::vec3(1, 0, 0);
        m.penetration = 1.0f;

        PositionalCorrection(A, B, m);

        EXPECT_NE(A.Position().x, 0.0f);
        ExpectVec3Near(B.Position(), glm::vec3(0, 0, 0), 1e-6f);
    }

    // ---- 5. Slop prevents tiny corrections ----
    {
        RigidBody A = MakeDynamicBody({ 0,0,0 }, 1.0f);
        RigidBody B = MakeDynamicBody({ 0,0,0 }, 1.0f);

        CollisionManifold m{};
        m.hit = true;
        m.normal = glm::vec3(1, 0, 0);
        m.penetration = 0.0005f; // less than default slop

        PositionalCorrection(A, B, m);

        ExpectVec3Near(A.Position(), glm::vec3(0, 0, 0), 1e-6f);
        ExpectVec3Near(B.Position(), glm::vec3(0, 0, 0), 1e-6f);
    }
}