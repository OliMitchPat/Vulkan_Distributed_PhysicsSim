// name=CppGoogleTest/OBB-Plane.cpp
#include "pch.h"
#include <gtest/gtest.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

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

TEST(Intersect_OBBPlane, NoHit_WhenSeparated)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) }; // y=0
    WorldOBB b = MakeAABB(glm::vec3(0, 5, 0), glm::vec3(1, 1, 1));

    // signedD = 5, r = 1 => abs(signedD) > r => no hit
    EXPECT_FALSE(Intersect(b, p).hit);
}

TEST(Intersect_OBBPlane, Hit_WhenOverlapping)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };
    WorldOBB b = MakeAABB(glm::vec3(0, 0.25f, 0), glm::vec3(1, 1, 1));

    // signedD = 0.25, r = 1 => penetration = 0.75
    CollisionManifold m = Intersect(b, p);
    ExpectManifoldHitInvariants(m);

    // signedD >= 0 => normal = +Y
    ExpectVec3Near(m.normal, glm::vec3(0, 1, 0), 1e-4f);
    EXPECT_NEAR(m.penetration, 0.75f, 1e-4f);

    // contact point should lie on plane y=0 (projection of center)
    EXPECT_NEAR(m.contactPoint.y, 0.0f, 1e-4f);
}

TEST(Intersect_OBBPlane, Boundary_PrecisionJustInsideAndOutside)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };
    const glm::vec3 halfE{ 1,1,1 };

    // r onto plane normal is 1 for this AABB
    WorldOBB inside = MakeAABB(glm::vec3(0, 1.0f - 1e-6f, 0), halfE);
    WorldOBB outside = MakeAABB(glm::vec3(0, 1.0f + 1e-6f, 0), halfE);

    EXPECT_TRUE(Intersect(inside, p).hit);
    EXPECT_FALSE(Intersect(outside, p).hit);
}

TEST(Intersect_OBBPlane, Hit_WithRotatedOBB_StillComputesProjectionRadius)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };

    WorldOBB b{};
    b.center = glm::vec3(0, 0.25f, 0);
    b.halfExtents = glm::vec3(1, 1, 1);

    // rotate 45 degrees about Z so axisX/axisY rotate in XY plane
    const float ang = glm::radians(45.0f);
    const glm::mat3 R = glm::mat3(glm::rotate(glm::mat4(1.0f), ang, glm::vec3(0, 0, 1)));
    b.axisX = R[0];
    b.axisY = R[1];
    b.axisZ = R[2];

    CollisionManifold m = Intersect(b, p);
    ASSERT_TRUE(m.hit);

    // Normal should still be +/-Y (plane normal or flipped)
    ExpectUnitOrFallbackNormal(m.normal);
    EXPECT_NEAR(std::abs(m.normal.y), 1.0f, 1e-4f);
    EXPECT_GE(m.penetration, 0.0f);
}