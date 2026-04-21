#include "pch.h"
#include <gtest/gtest.h>

#include <glm/glm.hpp>

#include "WorldShapes.h"
#include "Intersect.h"
#include "TestUtils.h"

TEST(Intersect_CapsuleCylinder, NoHit_WhenAxesFarApart)
{
    WorldCapsule c{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };
    WorldCylinder y{ glm::vec3(5,0,0), glm::vec3(5,2,0), 0.5f };

    EXPECT_FALSE(Intersect(c, y).hit);
}

TEST(Intersect_CapsuleCylinder, Hit_WhenParallelAxesClose)
{
    WorldCapsule c{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };
    WorldCylinder y{ glm::vec3(0.75f,0,0), glm::vec3(0.75f,2,0), 0.5f };

    CollisionManifold m = Intersect(c, y);
    ExpectManifoldHitInvariants(m);

    ExpectVec3Near(m.normal, glm::vec3(1, 0, 0), 1e-4f);
    EXPECT_NEAR(m.penetration, 1.0f - 0.75f, 1e-4f);
}

TEST(Intersect_CapsuleCylinder, Boundary_PrecisionJustInsideAndOutside)
{
    WorldCapsule c{ glm::vec3(0,0,0), glm::vec3(0,2,0), 0.5f };

    WorldCylinder inside{ glm::vec3(0.999999f,0,0), glm::vec3(0.999999f,2,0), 0.5f };
    WorldCylinder outside{ glm::vec3(1.000001f,0,0), glm::vec3(1.000001f,2,0), 0.5f };

    EXPECT_TRUE(Intersect(c, inside).hit);
    EXPECT_FALSE(Intersect(c, outside).hit);
}