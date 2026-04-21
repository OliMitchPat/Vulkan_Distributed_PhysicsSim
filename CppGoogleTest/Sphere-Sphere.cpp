#include "pch.h" 
#include <gtest/gtest.h>

#include <glm/glm.hpp>

#include "WorldShapes.h"
#include "Intersect.h"
#include "CollisionManifold.h"

#include "TestUtils.h"

TEST(Intersect_SphereSphere, NoHit_WhenSeparated)
{
    WorldSphere a{ glm::vec3(0,0,0), 1.0f };
    WorldSphere b{ glm::vec3(3.1f,0,0), 1.0f };

    CollisionManifold m = Intersect(a, b);
    EXPECT_FALSE(m.hit);
}

TEST(Intersect_SphereSphere, Hit_WhenOverlapping)
{
    WorldSphere a{ glm::vec3(0,0,0), 1.0f };
    WorldSphere b{ glm::vec3(1.5f,0,0), 1.0f };

    CollisionManifold m = Intersect(a, b);

    ExpectManifoldHitInvariants(m);

    ExpectVec3Near(m.normal, glm::vec3(1, 0, 0), 1e-4f);
    EXPECT_NEAR(m.penetration, 0.5f, 1e-4f);
    ExpectVec3Near(m.contactPoint, glm::vec3(1, 0, 0), 1e-4f);
}

TEST(Intersect_SphereSphere, NormalFallback_WhenSameCenter)
{
    WorldSphere a{ glm::vec3(0,0,0), 1.0f };
    WorldSphere b{ glm::vec3(0,0,0), 1.0f };

    CollisionManifold m = Intersect(a, b);
    ASSERT_TRUE(m.hit);

    ExpectVec3Near(m.normal, glm::vec3(1, 0, 0), 1e-4f);
    EXPECT_NEAR(m.penetration, 2.0f, 1e-4f);
}

TEST(Intersect_SphereSphere, Hit_DiagonalNormalCorrect)
{
    WorldSphere a{ glm::vec3(0,0,0), 3.0f };
    WorldSphere b{ glm::vec3(3,4,0), 3.0f }; // dist=5, rSum=6 => penetration=1

    CollisionManifold m = Intersect(a, b);
    ExpectManifoldHitInvariants(m);

    ExpectVec3Near(m.normal, glm::vec3(0.6f, 0.8f, 0.0f), 1e-4f);
    EXPECT_NEAR(m.penetration, 1.0f, 1e-4f);
}

TEST(Intersect_SphereSphere, Hit_NonOriginCenters)
{
    WorldSphere a{ glm::vec3(2,3,-1), 2.5f };
    WorldSphere b{ glm::vec3(5,6,-2), 2.5f }; // dist=sqrt(19)~4.3589, rSum=5 => hit

    CollisionManifold m = Intersect(a, b);
    ExpectManifoldHitInvariants(m);
    EXPECT_GT(m.penetration, 0.0f);
}

TEST(Intersect_SphereSphere, Boundary_PrecisionInsideOutside)
{
    WorldSphere a{ glm::vec3(0,0,0), 5.0f };
    WorldSphere bInside{ glm::vec3(9.999999f, 0, 0), 5.0f }; // rSum=10, dist slightly inside
    WorldSphere bOutside{ glm::vec3(10.000001f, 0, 0), 5.0f }; // slightly outside

    EXPECT_TRUE(Intersect(a, bInside).hit);
    EXPECT_FALSE(Intersect(a, bOutside).hit);
}