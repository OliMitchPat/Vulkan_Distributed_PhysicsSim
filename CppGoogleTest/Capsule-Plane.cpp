// name=CppGoogleTest/Capsule-Plane.cpp
#include "pch.h"
#include <gtest/gtest.h>

#include <glm/glm.hpp>

#include "WorldShapes.h"
#include "Intersect.h"
#include "TestUtils.h"

TEST(Intersect_CapsulePlane, NoHit_WhenAboveByMoreThanRadius)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) }; // y=0
    WorldCapsule c{ glm::vec3(0, 2, 0), glm::vec3(0, 4, 0), 0.5f }; // closest endpoint y=2 => absD=2 > r

    EXPECT_FALSE(Intersect(c, p).hit);
}

TEST(Intersect_CapsulePlane, Hit_WhenEndpointWithinRadius_FromPositiveSide)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };
    WorldCapsule c{ glm::vec3(0, 0.25f, 0), glm::vec3(0, 2.0f, 0), 0.5f };

    CollisionManifold m = Intersect(c, p);
    ExpectManifoldHitInvariants(m);

    // closest endpoint is above plane => signedDClosest positive => normal = +Y
    ExpectVec3Near(m.normal, glm::vec3(0, 1, 0), 1e-4f);
    EXPECT_NEAR(m.penetration, 0.5f - 0.25f, 1e-4f);
    EXPECT_NEAR(m.contactPoint.y, 0.0f, 1e-4f);
}

TEST(Intersect_CapsulePlane, Hit_WhenEndpointWithinRadius_FromNegativeSide_NormalFlips)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };
    WorldCapsule c{ glm::vec3(0, -0.25f, 0), glm::vec3(0, -2.0f, 0), 0.5f };

    CollisionManifold m = Intersect(c, p);
    ExpectManifoldHitInvariants(m);

    // signedDClosest negative => normal = -Y
    ExpectVec3Near(m.normal, glm::vec3(0, -1, 0), 1e-4f);
}

TEST(Intersect_CapsulePlane, Hit_WhenSegmentCrossesPlane_PenetrationEqualsRadius)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };

    // Endpoints on opposite sides => closest signed distance is 0
    WorldCapsule c{ glm::vec3(0, -1, 0), glm::vec3(0, 1, 0), 0.5f };

    CollisionManifold m = Intersect(c, p);
    ExpectManifoldHitInvariants(m);

    EXPECT_NEAR(m.penetration, 0.5f, 1e-4f);
    EXPECT_NEAR(m.contactPoint.y, 0.0f, 1e-4f);
}

TEST(Intersect_CapsulePlane, Boundary_PrecisionJustInsideAndOutside)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };

    // Choose capsule endpoints both above plane; closest endpoint defines absD.
    WorldCapsule inside{ glm::vec3(0, 0.499999f, 0), glm::vec3(0, 2, 0), 0.5f };
    WorldCapsule outside{ glm::vec3(0, 0.500001f, 0), glm::vec3(0, 2, 0), 0.5f };

    EXPECT_TRUE(Intersect(inside, p).hit);
    EXPECT_FALSE(Intersect(outside, p).hit);
}