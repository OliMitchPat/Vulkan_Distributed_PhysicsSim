// name=CppGoogleTest/Containment-Plane.cpp
#include "pch.h"
#include <gtest/gtest.h>
#include <glm/glm.hpp>

#include "WorldShapes.h"
#include "Containment.h"
#include "TestUtils.h"

TEST(Containment_Plane, Hit_CapsuleOutside)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) }; // inside is y<=0
    WorldCapsule a{ glm::vec3(0, 0.25f, 0), glm::vec3(0, 1.0f, 0), 0.5f }; // maxEnd=1 => maxD=1.5

    CollisionManifold m = Contain(a, p);
    ExpectManifoldHitInvariants(m);
    ExpectVec3Near(m.normal, glm::vec3(0, -1, 0), 1e-4f);
    EXPECT_GT(m.penetration, 0.0f);
}

TEST(Containment_Plane, Hit_CylinderOutside)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };
    WorldCylinder a{ glm::vec3(0, 0.25f, 0), glm::vec3(0, 1.0f, 0), 0.5f };

    CollisionManifold m = Contain(a, p);
    ExpectManifoldHitInvariants(m);
    ExpectVec3Near(m.normal, glm::vec3(0, -1, 0), 1e-4f);
    EXPECT_GT(m.penetration, 0.0f);
}

TEST(Containment_Plane, Hit_OBBOutside)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };

    WorldOBB a{};
    a.center = glm::vec3(0, 0.25f, 0);
    a.axisX = glm::vec3(1, 0, 0);
    a.axisY = glm::vec3(0, 1, 0);
    a.axisZ = glm::vec3(0, 0, 1);
    a.halfExtents = glm::vec3(1, 1, 1); // projection radius onto +Y is 1

    // centerD=0.25, r=1 => maxD=1.25 => outside
    CollisionManifold m = Contain(a, p);
    ExpectManifoldHitInvariants(m);
    ExpectVec3Near(m.normal, glm::vec3(0, -1, 0), 1e-4f);
    EXPECT_NEAR(m.penetration, 1.25f, 1e-4f);
}

TEST(Containment_Plane, Boundary_PrecisionSphereJustInsideAndOutside)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };

    // maxD = centerD + radius. Inside requires <= 0
    WorldSphere inside{ glm::vec3(0, -1.000001f, 0), 1.0f }; // maxD ~ -0.000001 => inside
    WorldSphere outside{ glm::vec3(0, -0.999999f, 0), 1.0f }; // maxD ~ +0.000001 => outside

    EXPECT_FALSE(Contain(inside, p).hit);
    EXPECT_TRUE(Contain(outside, p).hit);
}