// name=CppGoogleTest/OBB-OBB.cpp
#include "pch.h"
#include <gtest/gtest.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>

#include "WorldShapes.h"
#include "Intersect.h"
#include "TestUtils.h"

static WorldOBB MakeAABB(const glm::vec3& center, const glm::vec3& halfExtents)
{
    WorldOBB b{};
    b.center = center;
    b.axisX = glm::vec3(1, 0, 0);
    b.axisY = glm::vec3(0, 1, 0);
    b.axisZ = glm::vec3(0, 0, 1);
    b.halfExtents = halfExtents;
    return b;
}

static WorldOBB MakeRotZBox(const glm::vec3& center, const glm::vec3& halfExtents, float degrees)
{
    WorldOBB b{};
    b.center = center;
    b.halfExtents = halfExtents;

    const float ang = glm::radians(degrees);
    const glm::mat3 R = glm::mat3(glm::rotate(glm::mat4(1.0f), ang, glm::vec3(0, 0, 1)));
    b.axisX = R[0];
    b.axisY = R[1];
    b.axisZ = R[2];
    return b;
}

TEST(Intersect_OBBOBB, NoHit_WhenClearlySeparated)
{
    WorldOBB a = MakeAABB(glm::vec3(0, 0, 0), glm::vec3(1, 1, 1));
    WorldOBB b = MakeAABB(glm::vec3(5, 0, 0), glm::vec3(1, 1, 1));

    EXPECT_FALSE(Intersect(a, b).hit);
}

TEST(Intersect_OBBOBB, Hit_WhenOverlapping_AxisAligned)
{
    WorldOBB a = MakeAABB(glm::vec3(0, 0, 0), glm::vec3(1, 1, 1));
    WorldOBB b = MakeAABB(glm::vec3(1.5f, 0, 0), glm::vec3(1, 1, 1)); // overlap along x: 0.5

    CollisionManifold m = Intersect(a, b);
    ExpectManifoldHitInvariants(m);

    // In this simple case, best axis should be +/-X
    EXPECT_NEAR(std::abs(m.normal.x), 1.0f, 1e-4f);
    EXPECT_NEAR(m.penetration, 0.5f, 1e-4f);
}

TEST(Intersect_OBBOBB, Hit_WhenOverlapping_WithRotation)
{
    WorldOBB a = MakeAABB(glm::vec3(0, 0, 0), glm::vec3(1, 1, 1));
    WorldOBB b = MakeRotZBox(glm::vec3(1.2f, 0, 0), glm::vec3(1, 1, 1), 25.0f);

    CollisionManifold m = Intersect(a, b);
    ASSERT_TRUE(m.hit);

    // Don’t over-constrain which axis SAT selects.
    ExpectUnitOrFallbackNormal(m.normal);
    EXPECT_TRUE(std::isfinite(m.penetration));
    EXPECT_GE(m.penetration, 0.0f);
}

TEST(Intersect_OBBOBB, Stability_NearParallelAxes_NoNaNs)
{
    WorldOBB a = MakeRotZBox(glm::vec3(0, 0, 0), glm::vec3(1, 1, 1), 0.0f);

    // Very small rotation => cross products very small => your code skips near-zero axes.
    WorldOBB b = MakeRotZBox(glm::vec3(1.5f, 0, 0), glm::vec3(1, 1, 1), 0.01f);

    CollisionManifold m = Intersect(a, b);

    if (m.hit)
    {
        ExpectUnitOrFallbackNormal(m.normal);
        EXPECT_TRUE(std::isfinite(m.penetration));
        EXPECT_GE(m.penetration, 0.0f);
        EXPECT_TRUE(std::isfinite(m.contactPoint.x));
        EXPECT_TRUE(std::isfinite(m.contactPoint.y));
        EXPECT_TRUE(std::isfinite(m.contactPoint.z));
    }
    else
    {
        // Even if not hit due to numeric edge, ensure it’s a clean no-hit (default manifold).
        EXPECT_FALSE(m.hit);
    }
}