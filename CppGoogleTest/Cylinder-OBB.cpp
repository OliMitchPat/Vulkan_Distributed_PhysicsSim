// name=CppGoogleTest/Cylinder-OBB.cpp
#include "pch.h"
#include <gtest/gtest.h>

#include <glm/glm.hpp>

#include "WorldShapes.h"
#include "Intersect.h"
#include "TestUtils.h"

static WorldOBB MakeUnitAABB()
{
    WorldOBB b{};
    b.center = glm::vec3(0, 0, 0);
    b.axisX = glm::vec3(1, 0, 0);
    b.axisY = glm::vec3(0, 1, 0);
    b.axisZ = glm::vec3(0, 0, 1);
    b.halfExtents = glm::vec3(1, 1, 1);
    return b;
}

TEST(Intersect_CylinderOBB, NoHit_WhenFarAway)
{
    WorldOBB b = MakeUnitAABB();
    WorldCylinder c{ glm::vec3(5,0,0), glm::vec3(5,2,0), 0.5f };

    EXPECT_FALSE(Intersect(c, b).hit);
}

TEST(Intersect_CylinderOBB, Hit_WhenEndpointNearFace)
{
    WorldOBB b = MakeUnitAABB();

    WorldCylinder c{ glm::vec3(1.25f, 0, 0), glm::vec3(1.25f, 1, 0), 0.5f };

    CollisionManifold m = Intersect(c, b);
    ExpectManifoldHitInvariants(m);

    EXPECT_GT(m.normal.x, 0.7f);
    EXPECT_NEAR(m.contactPoint.x, 1.0f, 1e-3f);
}

TEST(Intersect_CylinderOBB, Hit_DiagonalNearCorner)
{
    WorldOBB b = MakeUnitAABB();

    WorldCylinder c{ glm::vec3(1.2f, 1.2f, 1.2f), glm::vec3(1.2f, 2.0f, 1.2f), 0.5f };

    CollisionManifold m = Intersect(c, b);
    ExpectManifoldHitInvariants(m);

    EXPECT_GT(m.normal.x, 0.0f);
    EXPECT_GT(m.normal.y, 0.0f);
    EXPECT_GT(m.normal.z, 0.0f);
}

TEST(Intersect_CylinderOBB, Boundary_PrecisionJustInsideAndOutside_Face)
{
    WorldOBB b = MakeUnitAABB();

    // boundary is x == 1 + radius = 1.5
    WorldCylinder inside{ glm::vec3(1.5f - 1e-6f, 0, 0), glm::vec3(1.5f - 1e-6f, 1, 0), 0.5f };
    WorldCylinder outside{ glm::vec3(1.5f + 1e-6f, 0, 0), glm::vec3(1.5f + 1e-6f, 1, 0), 0.5f };

    EXPECT_TRUE(Intersect(inside, b).hit);
    EXPECT_FALSE(Intersect(outside, b).hit);
}