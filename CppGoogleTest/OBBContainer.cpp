// name=CppGoogleTest/Containment-OBB.cpp
#include "pch.h"
#include <gtest/gtest.h>
#include <glm/glm.hpp>

#include "WorldShapes.h"
#include "Containment.h"
#include "TestUtils.h"

static WorldOBB MakeUnitAABB()
{
    WorldOBB c{};
    c.center = glm::vec3(0, 0, 0);
    c.axisX = glm::vec3(1, 0, 0);
    c.axisY = glm::vec3(0, 1, 0);
    c.axisZ = glm::vec3(0, 0, 1);
    c.halfExtents = glm::vec3(1, 1, 1);
    return c;
}

TEST(Containment_OBBContainer, NoHit_SphereInside)
{
    WorldOBB c = MakeUnitAABB();
    WorldSphere a{ glm::vec3(0,0,0), 0.25f };

    EXPECT_FALSE(Contain(a, c).hit);
}

TEST(Containment_OBBContainer, Hit_SphereOutsideFace)
{
    WorldOBB c = MakeUnitAABB();
    WorldSphere a{ glm::vec3(0.9f, 0, 0), 0.25f }; // extends past +X face at x=1

    CollisionManifold m = Contain(a, c);
    ExpectManifoldHitInvariants(m);

    // should push inward: when x>=0, normal = -axisX
    ExpectVec3Near(m.normal, glm::vec3(-1, 0, 0), 1e-4f);
    EXPECT_GT(m.penetration, 0.0f);
}

TEST(Containment_OBBContainer, Hit_CapsuleOutside_WorstEndpoint)
{
    WorldOBB c = MakeUnitAABB();
    WorldCapsule a{ glm::vec3(0.9f,0,0), glm::vec3(0,0,0), 0.25f };

    CollisionManifold m = Contain(a, c);
    ASSERT_TRUE(m.hit);
    ExpectUnitOrFallbackNormal(m.normal);
    EXPECT_GT(m.penetration, 0.0f);
}

TEST(Containment_OBBContainer, Hit_OBBOutside_OneCorner)
{
    WorldOBB c = MakeUnitAABB();

    WorldOBB a = MakeUnitAABB();
    a.center = glm::vec3(1.25f, 0, 0); // shifted so corners violate +X

    CollisionManifold m = Contain(a, c);
    ASSERT_TRUE(m.hit);
    ExpectUnitOrFallbackNormal(m.normal);
    EXPECT_GT(m.penetration, 0.0f);
}