#include "pch.h"

#include "Sphere.h"
#include "Plane.h"
#include "Line.h"
#include <stdexcept>

TEST(IsInside, InsideCases)
{
    EXPECT_TRUE(Sphere({ 0, 0, 0 }, 5).IsInside({ 0, 0, 0 }));                 // Basic - Centre
    EXPECT_TRUE(Sphere({ 0, 0, 0 }, 5).IsInside({ 3, 4, 0 }));                 // Diagonal - Inside (on surface)
    EXPECT_TRUE(Sphere({ 2, 3, -1 }, 10).IsInside({ 5, 6, -2 }));              // Non-Origin - Inside
    EXPECT_TRUE(Sphere({ 0, 0, 0 }, 5).IsInside({ 4.999999, 0, 0 }));          // Precision - Inside
    EXPECT_TRUE(Sphere({ -2, -3, -4 }, 7).IsInside({ -5, -6, -4 }));           // Negative - Inside
    EXPECT_TRUE(Sphere({ 7, 8, 9 }, 10).IsInside({ 16.99, 8, 9 }));            // Close Call - Inside
}

TEST(IsInside, OutsideCases)
{
    EXPECT_FALSE(Sphere({ 0, 0, 0 }, 5).IsInside({ 6, 0, 0 }));                // Basic - Outside
    EXPECT_FALSE(Sphere({ 0, 0, 0 }, 5).IsInside({ 4, 4, 0 }));                // Diagonal - Outside
    EXPECT_FALSE(Sphere({ 2, 3, -1 }, 10).IsInside({ 15, 3, -1 }));            // Non-Origin - Outside
    EXPECT_FALSE(Sphere({ 0, 0, 0 }, 5).IsInside({ 5.000001, 0, 0 }));         // Precision - Outside
    EXPECT_FALSE(Sphere({ -2, -3, -4 }, 7).IsInside({ -10, -3, -4 }));         // Negative - Outside
    EXPECT_FALSE(Sphere({ 7, 8, 9 }, 10).IsInside({ 17.01, 8, 9 }));           // Close Call - Outside
}

TEST(SphereConstructor, NegativeRadiusThrows)
{
    EXPECT_THROW(Sphere({ 0, 0, 0 }, -1), std::invalid_argument);
}

TEST(SphereIntersectsLine, PassesThroughSphere)
{
    // Sphere at origin radius 5. Segment crosses x-axis from -10 to +10.
    // Closest approach to centre is 0 <= 5 => intersects.
    Sphere s({ 0,0,0 }, 5);
    Line l({ -10,0,0 }, { 10,0,0 });
    EXPECT_TRUE(s.Intersects(l));
}

TEST(SphereIntersectsLine, MissesSphere)
{
    // Entire segment is outside and does not cross into sphere volume.
    Sphere s({ 0,0,0 }, 5);
    Line l({ 6,0,0 }, { 10,0,0 });
    EXPECT_FALSE(s.Intersects(l));
}

TEST(SphereIntersectsLine, TangentCountsAsIntersect)
{
    // Line at y=5 touches sphere of radius 5 at (0,5,0).
    Sphere s({ 0,0,0 }, 5);
    Line l({ -10,5,0 }, { 10,5,0 });
    EXPECT_TRUE(s.Intersects(l));
}

TEST(PlaneIsInside, NormalSideAndOnPlane)
{
    // Plane through origin with normal +Y. Inside means y >= 0.
    Plane p({ 0,0,0 }, { 0,1,0 });
    EXPECT_TRUE(p.IsInside({ 0, 1, 0 }));  // above plane
    EXPECT_TRUE(p.IsInside({ 0, 0, 0 }));  // on plane
    EXPECT_FALSE(p.IsInside({ 0,-1, 0 })); // below plane
}

TEST(PlaneIntersectsLine, CrossesPlane)
{
    // Segment from y=-1 to y=+1 crosses the plane y=0.
    Plane p({ 0,0,0 }, { 0,1,0 });
    Line l({ 0,-1,0 }, { 0, 1,0 });
    EXPECT_TRUE(p.Intersects(l));
}

TEST(PlaneIntersectsLine, ParallelNoIntersection)
{
    // Segment entirely at y=1 (parallel to plane y=0) => does not intersect plane.
    Plane p({ 0,0,0 }, { 0,1,0 });
    Line l({ 0,1,0 }, { 10,1,0 });
    EXPECT_FALSE(p.Intersects(l));
}

TEST(PlaneIntersectsLine, LiesInPlane)
{
    // Segment is on plane y=0 => intersects (infinite points).
    Plane p({ 0,0,0 }, { 0,1,0 });
    Line l({ 0,0,0 }, { 10,0,0 });
    EXPECT_TRUE(p.Intersects(l));
}

TEST(ConstructorGuards, SphereNegativeRadiusThrows)
{
    EXPECT_THROW(Sphere({ 0,0,0 }, -1), std::invalid_argument);
}

TEST(ConstructorGuards, PlaneZeroNormalThrows)
{
    EXPECT_THROW(Plane({ 0,0,0 }, { 0,0,0 }), std::invalid_argument);
}
