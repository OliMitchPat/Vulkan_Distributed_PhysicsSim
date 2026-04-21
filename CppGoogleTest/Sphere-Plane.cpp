#include "pch.h" 
#include <gtest/gtest.h>
#include "WorldShapes.h"
#include "Intersect.h"
#include "TestUtils.h"

TEST(Intersect_SpherePlane, NoHit_WhenAboveByMoreThanRadius)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) }; // plane y=0
    WorldSphere s{ glm::vec3(0, 2.1f, 0), 1.0f };       // distance 2.1 > 1

    CollisionManifold m = Intersect(s, p);
    EXPECT_FALSE(m.hit);
}

TEST(Intersect_SpherePlane, Hit_WhenIntersecting_FromPositiveSide)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };
    WorldSphere s{ glm::vec3(0, 0.25f, 0), 1.0f }; // penetration 0.75

    CollisionManifold m = Intersect(s, p);
    ExpectManifoldHitInvariants(m);

    // normal points into half-space where sphere center lies: +Y
    ExpectVec3Near(m.normal, glm::vec3(0, 1, 0), 1e-4f);
    EXPECT_NEAR(m.penetration, 0.75f, 1e-4f);

    // closest point on plane: y=0
    ExpectVec3Near(m.contactPoint, glm::vec3(0, 0, 0), 1e-4f);
}

TEST(Intersect_SpherePlane, Hit_WhenIntersecting_FromNegativeSide_NormalFlips)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };
    WorldSphere s{ glm::vec3(0, -0.25f, 0), 1.0f };

    CollisionManifold m = Intersect(s, p);
    ExpectManifoldHitInvariants(m);

    // sphere center is below plane => signedD negative => normal becomes -Y
    ExpectVec3Near(m.normal, glm::vec3(0, -1, 0), 1e-4f);
}

TEST(Intersect_SpherePlane, Hit_WhenCenterOnPlane)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };
    WorldSphere s{ glm::vec3(2, 0, -3), 5.0f }; // non-origin

    CollisionManifold m = Intersect(s, p);
    ExpectManifoldHitInvariants(m);

    // signedD = 0 => penetration = radius
    EXPECT_NEAR(m.penetration, 5.0f, 1e-4f);
    ExpectVec3Near(m.contactPoint, glm::vec3(2, 0, -3), 1e-4f);
}

TEST(Intersect_SpherePlane, Boundary_PrecisionJustInsideAndOutside)
{
    WorldPlane p{ glm::vec3(0,0,0), glm::vec3(0,1,0) };
    const float r = 5.0f;

    WorldSphere inside{ glm::vec3(0, r - 1e-6f, 0), r };
    WorldSphere outside{ glm::vec3(0, r + 1e-6f, 0), r };

    EXPECT_TRUE(Intersect(inside, p).hit);
    EXPECT_FALSE(Intersect(outside, p).hit);
}

TEST(Intersect_SpherePlane, Hit_WithNonAxisAlignedPlaneNormal)
{
    // Plane through origin with normal at 45 degrees between +X and +Y.
    const glm::vec3 n = glm::normalize(glm::vec3(1, 1, 0));
    WorldPlane p{ glm::vec3(0,0,0), n };

    // Put sphere center along +n direction by 0.25, radius 1 => hit, normal should be +n
    WorldSphere s{ n * 0.25f, 1.0f };

    CollisionManifold m = Intersect(s, p);
    ExpectManifoldHitInvariants(m);

    ExpectVec3Near(m.normal, n, 1e-4f);
    EXPECT_NEAR(m.penetration, 0.75f, 1e-4f);
}