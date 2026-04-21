// name=CppGoogleTest/Cylinder-Cylinder.cpp
#include "pch.h"
#include <gtest/gtest.h>

#include <glm/glm.hpp>

#include "WorldShapes.h"
#include "Intersect.h"
#include "TestUtils.h"

TEST(Intersect_CylinderCylinder, NoHit_WhenFarApart)
{
    WorldCylinder a{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };
    WorldCylinder b{ glm::vec3(5,0,0), glm::vec3(5,2,0), 0.5f };

    EXPECT_FALSE(Intersect(a, b).hit);
}

TEST(Intersect_CylinderCylinder, Hit_WhenParallelAxesClose)
{
    WorldCylinder a{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };
    WorldCylinder b{ glm::vec3(0.75f,0,0), glm::vec3(0.75f,2,0), 0.5f };

    CollisionManifold m = Intersect(a, b);
    ExpectManifoldHitInvariants(m);

    ExpectVec3Near(m.normal, glm::vec3(1, 0, 0), 1e-4f);
    EXPECT_NEAR(m.penetration, 1.0f - 0.75f, 1e-4f);
}

TEST(Intersect_CylinderCylinder, Boundary_PrecisionJustInsideAndOutside)
{
    WorldCylinder a{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };

    WorldCylinder inside{ glm::vec3(0.999999f,0,0), glm::vec3(0.999999f,2,0), 0.5f };
    WorldCylinder outside{ glm::vec3(1.000001f,0,0), glm::vec3(1.000001f,2,0), 0.5f };

    EXPECT_TRUE(Intersect(a, inside).hit);
    EXPECT_FALSE(Intersect(a, outside).hit);
}

TEST(Intersect_CylinderCylinder, Edge_DegenerateSameAxisSamePlace)
{
    WorldCylinder a{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };
    WorldCylinder b{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };

    CollisionManifold m = Intersect(a, b);
    ASSERT_TRUE(m.hit);

    // Expect stable manifold (this will fail until you apply the same distSq-degenerate fix).
    ExpectUnitOrFallbackNormal(m.normal);
    EXPECT_GT(m.penetration, 0.0f);
}