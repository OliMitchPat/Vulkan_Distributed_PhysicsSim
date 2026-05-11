// name=CppGoogleTest/Containment-Sphere.cpp
#include "pch.h"
#include <gtest/gtest.h>
#include <glm/glm.hpp>

#include "WorldShapes.h"
#include "Containment.h"
#include "RigidBody.h"
#include "PositionalCorrection.h"
#include "TestUtils.h"

TEST(Containment_SphereContainer, NoHit_SphereFullyInside)
{
    WorldSphere c{ glm::vec3(0,0,0), 10.0f };
    WorldSphere a{ glm::vec3(0,0,0), 1.0f };

    EXPECT_FALSE(Contain(a, c).hit);
}

TEST(Containment_SphereContainer, Hit_SphereOutside)
{
    WorldSphere c{ glm::vec3(0,0,0), 10.0f };
    WorldSphere a{ glm::vec3(9.5f,0,0), 1.0f }; // allowed center distance = 9

    CollisionManifold m = Contain(a, c);
    ExpectManifoldHitInvariants(m);

    ExpectVec3Near(m.normal, glm::vec3(-1, 0, 0), 1e-4f); // push back toward center
    EXPECT_NEAR(m.penetration, 0.5f, 1e-4f);
}

TEST(Containment_SphereContainer, FlippedNormalMatchesResolveContactCorrectionDirection)
{
    // Outside on +X: Contain() returns inward normal (-X). CollisionSystem flips it
    // before ResolveContact so positional correction moves the inner body inward.
    WorldSphere container{ glm::vec3(0,0,0), 10.0f };
    WorldSphere inner{ glm::vec3(9.5f,0,0), 1.0f };

    CollisionManifold m = Contain(inner, container);
    ASSERT_TRUE(m.hit);

    RigidBody innerBody;
    innerBody.SetMotionType(BodyMotionType::Dynamic);
    innerBody.SetMass(1.0f);
    innerBody.SetPosition(inner.center);

    RigidBody containerBody;
    containerBody.SetMotionType(BodyMotionType::Static);
    containerBody.SetPosition(container.center);

    m.normal = -m.normal; // same convention fix applied in CollisionSystem::ContainInContainer
    PositionalCorrection(innerBody, containerBody, m, 1.0f, 0.0f);

    EXPECT_LT(innerBody.Position().x, inner.center.x);
}

TEST(Containment_SphereContainer, Hit_CapsuleOutside_UsesWorstEndpoint)
{
    WorldSphere c{ glm::vec3(0,0,0), 10.0f };
    WorldCapsule a{ glm::vec3(9.5f,0,0), glm::vec3(0,0,0), 1.0f }; // worst endpoint at 9.5 => maxD=10.5

    CollisionManifold m = Contain(a, c);
    ASSERT_TRUE(m.hit);
    ExpectUnitOrFallbackNormal(m.normal);
    EXPECT_GT(m.penetration, 0.0f);
}

TEST(Containment_SphereContainer, Hit_OBBOutside_UsesWorstCorner)
{
    WorldSphere c{ glm::vec3(0,0,0), 5.0f };

    WorldOBB a{};
    a.center = glm::vec3(4.5f, 0, 0);
    a.axisX = glm::vec3(1, 0, 0);
    a.axisY = glm::vec3(0, 1, 0);
    a.axisZ = glm::vec3(0, 0, 1);
    a.halfExtents = glm::vec3(1, 1, 1); // worst corner distance will exceed 5

    CollisionManifold m = Contain(a, c);
    ASSERT_TRUE(m.hit);
    ExpectUnitOrFallbackNormal(m.normal);
    EXPECT_GT(m.penetration, 0.0f);
}
