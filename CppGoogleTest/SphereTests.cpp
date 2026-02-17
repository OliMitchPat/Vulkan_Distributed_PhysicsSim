#include "pch.h"
#include "Sphere.h"
#include "Plane.h"

TEST(SphereSphereCollision, NoIntersectionCentreAtOrigin)
{
    Sphere sphereA({ 0, 0, 0 }, 1);
    Sphere sphereB({ 5, 0, 0 }, 1);

    EXPECT_FALSE(sphereA.CollideWith(sphereB))
        << "Sphere at (0,0,0) r=1 should NOT collide with sphere at (5,0,0) r=1";
}

TEST(SphereSphereCollision, NoIntersectionOffsetCentre)
{
    Sphere sphereA({ 3, 3, 3 }, 2);
    Sphere sphereB({ 10, 10, 10 }, 2);

    EXPECT_FALSE(sphereA.CollideWith(sphereB));
}

TEST(SphereSphereCollision, OverlappingCentreAtOrigin)
{
    Sphere sphereA({ 0, 0, 0 }, 2);
    Sphere sphereB({ 2, 0, 0 }, 2);

    EXPECT_TRUE(sphereA.CollideWith(sphereB));
}

TEST(SphereSphereCollision, OverlappingOffsetCentre)
{
    Sphere sphereA({ 5, 5, 5 }, 3);
    Sphere sphereB({ 8, 5, 5 }, 3);

    EXPECT_TRUE(sphereA.CollideWith(sphereB));
}

TEST(SphereSphereCollision, FullyContainedCentreAtOrigin)
{
    Sphere sphereA({ 0, 0, 0 }, 3);
    Sphere sphereB({ 1, 0, 0 }, 1);

    EXPECT_TRUE(sphereA.CollideWith(sphereB));
}

TEST(SphereSphereCollision, FullyContainedOffsetCentre)
{
    Sphere sphereA({ 6, 6, 6 }, 5);
    Sphere sphereB({ 7, 6, 6 }, 2);

    EXPECT_TRUE(sphereA.CollideWith(sphereB));
}

TEST(SphereSphereCollision, IdenticalSpheresIntersect)
{
    Sphere sphereA({ 1, 2, 3 }, 4);
    Sphere sphereB({ 1, 2, 3 }, 4);

    EXPECT_TRUE(sphereA.CollideWith(sphereB));
}

TEST(SphereSphereCollision, JustTouchingCountsAsCollision)
{
    Sphere sphereA({ 0, 0, 0 }, 2);
    Sphere sphereB({ 4, 0, 0 }, 2);

    EXPECT_TRUE(sphereA.CollideWith(sphereB));
}

TEST(SphereLineIntersection, NoIntersectionCentreAtOrigin)
{
    // Line: point (5,5,5), direction (1,0,0)
    // Sphere: centre (0,0,0), r=3
    Sphere s({ 0,0,0 }, 3);
    EXPECT_FALSE(s.IntersectsInfiniteLine({ 5,5,5 }, { 1,0,0 }));
}

TEST(SphereLineIntersection, PassesThroughSphereCentreAtOrigin)
{
    // Line passes through sphere centre (10,0,0)
    Sphere s({ 10,0,0 }, 5);
    EXPECT_TRUE(s.IntersectsInfiniteLine({ 10,0,0 }, { -1,0,0 }));
}

TEST(SphereLineIntersection, LineStartsInsideSphere)
{
    Sphere s({ 2,2,2 }, 5);
    EXPECT_TRUE(s.IntersectsInfiniteLine({ 3,2,2 }, { 1,0,0 }));
}

TEST(SphereLineIntersection, PassesThroughSphereCenter)
{
    Sphere s({ 0,0,0 }, 3);
    EXPECT_TRUE(s.IntersectsInfiniteLine({ -5,0,0 }, { 1,0,0 }));
}

TEST(SphereLineIntersection, TangentLineCountsAsIntersect)
{
    Sphere s({ 0,0,0 }, 5);
    EXPECT_TRUE(s.IntersectsInfiniteLine({ 0,5,0 }, { 1,0,0 }));
}


TEST(SphereLineIntersection, ZeroDirectionThrows)
{
    Sphere s({ 0,0,0 }, 5);
    EXPECT_THROW(s.IntersectsInfiniteLine({ 0,0,0 }, { 0,0,0 }), std::invalid_argument);
}

TEST(SpherePlaneIntersection, AbovePlaneNoIntersection)
{
    Plane p({ 0,0,0 }, { 0,0,1 });
    Sphere s({ 0,0,5 }, 2);

    EXPECT_FALSE(p.Intersects(s));
}

TEST(SpherePlaneIntersection, TouchingPlane)
{
    Plane p({ 0,0,0 }, { 0,0,1 });
    Sphere s({ 0,0,3 }, 3);

    EXPECT_TRUE(p.Intersects(s));
}

TEST(SpherePlaneIntersection, CrossingPlane)
{
    Plane p({ 0,0,0 }, { 0,0,1 });
    Sphere s({ 0,0,1 }, 3);

    EXPECT_TRUE(p.Intersects(s));
}

TEST(SpherePlaneIntersection, SphereCentreOnPlane)
{
    Plane p({ 0,0,0 }, { 0,0,1 });
    Sphere s({ 5,5,0 }, 1);

    EXPECT_TRUE(p.Intersects(s));
}

TEST(SpherePlaneIntersection, NegativeSideStillIntersects)
{
    Plane p({ 0,0,0 }, { 0,0,1 });
    Sphere s({ 0,0,-2 }, 3);

    EXPECT_TRUE(p.Intersects(s));
}
