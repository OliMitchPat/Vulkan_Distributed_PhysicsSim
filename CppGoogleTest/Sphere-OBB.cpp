#include "pch.h" 
#include <gtest/gtest.h>
#include "WorldShapes.h"
#include "Intersect.h"
#include "TestUtils.h"

TEST(Intersect_SphereOBB, NoHit_WhenOutside)
{
    WorldOBB b{};
    b.center = glm::vec3(0, 0, 0);
    b.axisX = glm::vec3(1, 0, 0);
    b.axisY = glm::vec3(0, 1, 0);
    b.axisZ = glm::vec3(0, 0, 1);
    b.halfExtents = glm::vec3(1, 1, 1);

    WorldSphere s{ glm::vec3(3.0f, 0, 0), 0.5f };

    CollisionManifold m = Intersect(s, b);
    EXPECT_FALSE(m.hit);
}

TEST(Intersect_SphereOBB, Hit_WhenTouchingFace)
{
    WorldOBB b{};
    b.center = glm::vec3(0, 0, 0);
    b.axisX = glm::vec3(1, 0, 0);
    b.axisY = glm::vec3(0, 1, 0);
    b.axisZ = glm::vec3(0, 0, 1);
    b.halfExtents = glm::vec3(1, 1, 1);

    // Sphere center 1.25 from center => closest point on box is (1,0,0),
    // delta = (0.25,0,0), radius = 0.5 => penetration 0.25
    WorldSphere s{ glm::vec3(1.25f, 0, 0), 0.5f };

    CollisionManifold m = Intersect(s, b);
    ExpectManifoldHitInvariants(m);

    // normal points from box surface point toward sphere center => +X
    ExpectVec3Near(m.normal, glm::vec3(1, 0, 0), 1e-4f);
    EXPECT_NEAR(m.penetration, 0.25f, 1e-4f);
    ExpectVec3Near(m.contactPoint, glm::vec3(1, 0, 0), 1e-4f);
}

TEST(Intersect_SphereOBB, Hit_WhenSphereCenterInsideBox)
{
    WorldOBB b{};
    b.center = glm::vec3(0, 0, 0);
    b.axisX = glm::vec3(1, 0, 0);
    b.axisY = glm::vec3(0, 1, 0);
    b.axisZ = glm::vec3(0, 0, 1);
    b.halfExtents = glm::vec3(1, 1, 1);

    // Center is inside, radius small => should still be hit (distSq==0 <= r^2)
    WorldSphere s{ glm::vec3(0.25f, 0.0f, 0.0f), 0.1f };

    CollisionManifold m = Intersect(s, b);
    ASSERT_TRUE(m.hit);

    // Normal is not uniquely defined when inside; don't enforce direction.
    // But it must be unit (your code uses fallback axis if dist ~ 0).
    ExpectUnitOrFallbackNormal(m.normal);

    EXPECT_GE(m.penetration, 0.0f);
}

TEST(Intersect_SphereOBB, Boundary_PrecisionJustInsideAndOutside_Face)
{
    WorldOBB b{};
    b.center = glm::vec3(0, 0, 0);
    b.axisX = glm::vec3(1, 0, 0);
    b.axisY = glm::vec3(0, 1, 0);
    b.axisZ = glm::vec3(0, 0, 1);
    b.halfExtents = glm::vec3(1, 1, 1);

    // Closest point on box face is x=1
    // Choose radius=0.5. Then hit if center.x <= 1 + 0.5
    WorldSphere inside{ glm::vec3(1.5f - 1e-6f, 0, 0), 0.5f };
    WorldSphere outside{ glm::vec3(1.5f + 1e-6f, 0, 0), 0.5f };

    EXPECT_TRUE(Intersect(inside, b).hit);
    EXPECT_FALSE(Intersect(outside, b).hit);
}

TEST(Intersect_SphereOBB, Hit_DiagonalTowardCorner_NormalRoughlyDiagonal)
{
    WorldOBB b{};
    b.center = glm::vec3(0, 0, 0);
    b.axisX = glm::vec3(1, 0, 0);
    b.axisY = glm::vec3(0, 1, 0);
    b.axisZ = glm::vec3(0, 0, 1);
    b.halfExtents = glm::vec3(1, 1, 1);

    // Sphere near corner (1,1,1)
    WorldSphere s{ glm::vec3(1.2f, 1.2f, 1.2f), 0.5f };

    CollisionManifold m = Intersect(s, b);
    ExpectManifoldHitInvariants(m);

    // Closest point on box should be (1,1,1), delta should be approx (0.2,0.2,0.2)
    const glm::vec3 expectedN = glm::normalize(glm::vec3(0.2f, 0.2f, 0.2f));
    ExpectVec3Near(m.normal, expectedN, 1e-3f);
}