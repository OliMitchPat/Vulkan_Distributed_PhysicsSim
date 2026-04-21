// name=CppGoogleTest/Containment-Cylinder.cpp
#include "pch.h"
#include <gtest/gtest.h>
#include <glm/glm.hpp>

#include "WorldShapes.h"
#include "Containment.h"
#include "TestUtils.h"

static WorldCylinder MakeUprightCylinder(float radius, float height)
{
    WorldCylinder c{};
    c.a = glm::vec3(0, 0, 0);
    c.b = glm::vec3(0, height, 0);
    c.radius = radius;
    return c;
}

TEST(Containment_CylinderContainer, NoHit_SphereInside)
{
    WorldCylinder c = MakeUprightCylinder(5.0f, 10.0f);
    WorldSphere a{ glm::vec3(0,5,0), 1.0f };

    EXPECT_FALSE(Contain(a, c).hit);
}

TEST(Containment_CylinderContainer, Hit_SphereOutsideSideWall)
{
    WorldCylinder c = MakeUprightCylinder(5.0f, 10.0f);

    // At y=5, radial distance 4.5, radius 1 => needs radial <= 4.0, so outside by 0.5
    WorldSphere a{ glm::vec3(4.5f, 5, 0), 1.0f };

    CollisionManifold m = Contain(a, c);
    ExpectManifoldHitInvariants(m);

    // Side wall violation pushes inward toward axis => roughly -X
    EXPECT_LT(m.normal.x, -0.7f);
    EXPECT_NEAR(m.penetration, 0.5f, 1e-3f);
}

TEST(Containment_CylinderContainer, Hit_SphereOutsideCap)
{
    WorldCylinder c = MakeUprightCylinder(5.0f, 10.0f);

    // Must satisfy a.radius <= t <= height-a.radius.
    // Put center at y=0.5 with radius 1 => violates bottom cap by 0.5
    WorldSphere a{ glm::vec3(0, 0.5f, 0), 1.0f };

    CollisionManifold m = Contain(a, c);
    ExpectManifoldHitInvariants(m);

    // bottom cap => normal should push toward +axis (up)
    EXPECT_GT(m.normal.y, 0.7f);
    EXPECT_NEAR(m.penetration, 0.5f, 1e-3f);
}

TEST(Containment_CylinderContainer, Hit_CapsuleOutside_EndpointSphereMethod)
{
    WorldCylinder c = MakeUprightCylinder(5.0f, 10.0f);

    // endpoint sphere near side wall
    WorldCapsule a{ glm::vec3(4.5f, 5, 0), glm::vec3(0, 5, 0), 1.0f };

    CollisionManifold m = Contain(a, c);
    ASSERT_TRUE(m.hit);
    ExpectUnitOrFallbackNormal(m.normal);
    EXPECT_GT(m.penetration, 0.0f);
}

TEST(Containment_CylinderContainer, DegenerateHeight_DoesNotCrash)
{
    WorldCylinder c{ glm::vec3(0,0,0), glm::vec3(0,0,0), 5.0f };
    WorldSphere a{ glm::vec3(10,0,0), 1.0f };

    EXPECT_FALSE(Contain(a, c).hit);
}