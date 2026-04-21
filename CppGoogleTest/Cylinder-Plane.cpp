// name=CppGoogleTest/Cylinder-Plane.cpp
#include "pch.h"
#include <gtest/gtest.h>

#include <glm/glm.hpp>

#include "WorldShapes.h"
#include "Intersect.h"
#include "TestUtils.h"

TEST(Intersect_CylinderPlane, NoHit_WhenAboveByMoreThanRadius)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };
    WorldCylinder c{ glm::vec3(0, 2, 0), glm::vec3(0, 4, 0), 0.5f };

    EXPECT_FALSE(Intersect(c, p).hit);
}

TEST(Intersect_CylinderPlane, Hit_WhenEndpointWithinRadius_FromPositiveSide)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };

    // closest endpoint y=0.25 => penetration = 0.5 - 0.25 = 0.25
    WorldCylinder c{ glm::vec3(0, 0.25f, 0), glm::vec3(0, 2, 0), 0.5f };

    CollisionManifold m = Intersect(c, p);
    ExpectManifoldHitInvariants(m);

    ExpectVec3Near(m.normal, glm::vec3(0, 1, 0), 1e-4f);
    EXPECT_NEAR(m.penetration, 0.25f, 1e-4f);
    EXPECT_NEAR(m.contactPoint.y, 0.0f, 1e-4f);
}

TEST(Intersect_CylinderPlane, Hit_WhenEndpointWithinRadius_FromNegativeSide_NormalFlips)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };
    WorldCylinder c{ glm::vec3(0, -0.25f, 0), glm::vec3(0, -2, 0), 0.5f };

    CollisionManifold m = Intersect(c, p);
    ExpectManifoldHitInvariants(m);

    ExpectVec3Near(m.normal, glm::vec3(0, -1, 0), 1e-4f);
}

TEST(Intersect_CylinderPlane, Hit_WhenAxisCrossesPlane_PenetrationEqualsRadius)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };

    // axis crosses plane => signedDClosest=0 => penetration = radius
    WorldCylinder c{ glm::vec3(0, -1, 0), glm::vec3(0, 1, 0), 0.5f };

    CollisionManifold m = Intersect(c, p);
    ExpectManifoldHitInvariants(m);

    EXPECT_NEAR(m.penetration, 0.5f, 1e-4f);
    EXPECT_NEAR(m.contactPoint.y, 0.0f, 1e-4f);
}

TEST(Intersect_CylinderPlane, Boundary_PrecisionJustInsideAndOutside)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };

    WorldCylinder inside{ glm::vec3(0, 0.499999f, 0), glm::vec3(0, 2, 0), 0.5f };
    WorldCylinder outside{ glm::vec3(0, 0.500001f, 0), glm::vec3(0, 2, 0), 0.5f };

    EXPECT_TRUE(Intersect(inside, p).hit);
    EXPECT_FALSE(Intersect(outside, p).hit);
}