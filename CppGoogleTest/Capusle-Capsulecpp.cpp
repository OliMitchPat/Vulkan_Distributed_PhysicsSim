#include "pch.h"
#include <gtest/gtest.h>

#include <glm/glm.hpp>

#include "WorldShapes.h"
#include "Intersect.h"
#include "TestUtils.h"

TEST(Intersect_CapsuleCapsule, NoHit_WhenFarApart)
{
    WorldCapsule a{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };
    WorldCapsule b{ glm::vec3(5,0,0), glm::vec3(5,2,0), 0.5f };

    EXPECT_FALSE(Intersect(a, b).hit);
}

TEST(Intersect_CapsuleCapsule, Hit_WhenParallelSegmentsClose)
{
    WorldCapsule a{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };
    WorldCapsule b{ glm::vec3(0.75f,0,0), glm::vec3(0.75f,2,0), 0.5f }; // gap 0.75 < rSum 1.0

    CollisionManifold m = Intersect(a, b);
    ExpectManifoldHitInvariants(m);

    // Normal should point from A toward B (~+X)
    ExpectVec3Near(m.normal, glm::vec3(1, 0, 0), 1e-4f);
    EXPECT_NEAR(m.penetration, 1.0f - 0.75f, 1e-4f);
}

TEST(Intersect_CapsuleCapsule, Hit_DiagonalOffset)
{
    // Same direction, offset diagonally in XZ
    WorldCapsule a{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };
    WorldCapsule b{ glm::vec3(0.6f,0,0.8f), glm::vec3(0.6f,2,0.8f), 0.5f };

    // separation sqrt(0.6^2+0.8^2)=1.0, rSum=1.0 => touching => hit
    CollisionManifold m = Intersect(a, b);
    ExpectManifoldHitInvariants(m);

    // Normal should be ~ (0.6,0,0.8) normalized = (0.6,0,0.8)
    ExpectVec3Near(m.normal, glm::vec3(0.6f, 0.0f, 0.8f), 1e-4f);
    EXPECT_NEAR(m.penetration, 0.0f, 1e-4f);
}

TEST(Intersect_CapsuleCapsule, Boundary_PrecisionJustInsideAndOutside)
{
    WorldCapsule a{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };
    WorldCapsule inside{ glm::vec3(0.999999f,0,0), glm::vec3(0.999999f,2,0), 0.5f }; // rSum=1
    WorldCapsule outside{ glm::vec3(1.000001f,0,0), glm::vec3(1.000001f,2,0), 0.5f };

    EXPECT_TRUE(Intersect(a, inside).hit);
    EXPECT_FALSE(Intersect(a, outside).hit);
}

TEST(Intersect_CapsuleCapsule, Edge_DegenerateSegmentsSameLineSamePlace)
{
    WorldCapsule a{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };
    WorldCapsule b{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };

    CollisionManifold m = Intersect(a, b);
    ASSERT_TRUE(m.hit);

    ExpectVec3Near(m.normal, glm::vec3(1, 0, 0), 1e-4f);
    EXPECT_NEAR(m.penetration, 1.0f, 1e-4f); // rSum = 1.0
}