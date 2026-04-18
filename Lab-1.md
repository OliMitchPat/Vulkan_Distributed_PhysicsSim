# 700105 Simulation and Concurrency Lab Book

## Simulation Lab 1

*12/02/26*

### Q1. Sphere–Sphere Intersection Test

**Question:**

To check if two spheres intersect you must calculate the vector between their centres.
If the distance between the centres is less than or equal to the sum of the radii, the spheres intersect.
To avoid the expensive square root operation, compare the squared distance with the squared sum of the radii.

**Solution:**

```c++
bool Sphere::CollideWith(const Sphere& other) const
{
    const Vector3 delta = other.GetPosition() - this->GetPosition();
    const double rSum = this->GetRadius() + other.GetRadius();

    return delta.LengthSquared() <= (rSum * rSum);
}
```

**Test data:**
```c++
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
```
**Sample output:**

[----------] 8 tests from SphereSphereCollision
[ RUN      ] SphereSphereCollision.NoIntersectionCentreAtOrigin
[       OK ] SphereSphereCollision.NoIntersectionCentreAtOrigin (0 ms)
[ RUN      ] SphereSphereCollision.NoIntersectionOffsetCentre
[       OK ] SphereSphereCollision.NoIntersectionOffsetCentre (0 ms)
[ RUN      ] SphereSphereCollision.OverlappingCentreAtOrigin
[       OK ] SphereSphereCollision.OverlappingCentreAtOrigin (0 ms)
[ RUN      ] SphereSphereCollision.OverlappingOffsetCentre
[       OK ] SphereSphereCollision.OverlappingOffsetCentre (0 ms)
[ RUN      ] SphereSphereCollision.FullyContainedCentreAtOrigin
[       OK ] SphereSphereCollision.FullyContainedCentreAtOrigin (0 ms)
[ RUN      ] SphereSphereCollision.FullyContainedOffsetCentre
[       OK ] SphereSphereCollision.FullyContainedOffsetCentre (0 ms)
[ RUN      ] SphereSphereCollision.IdenticalSpheresIntersect
[       OK ] SphereSphereCollision.IdenticalSpheresIntersect (0 ms)
[ RUN      ] SphereSphereCollision.JustTouchingCountsAsCollision
[       OK ] SphereSphereCollision.JustTouchingCountsAsCollision (0 ms)
[----------] 8 tests from SphereSphereCollision (1 ms total)

**Reflection:**

In this task I implemented sphere–sphere collision detection using vector mathematics. To do this compare the squared distance between sphere centres to the sum of radii,
Squared length is used for a perfomance bonus as square roots are expensive.
Ive learned how to set up tests in google tests which I haven't used before and are helpful for testing if your code is correct without a visual demo.

**Questions:**

No

### Q2. Closest Distance from a Point to a Line 

An infinite line can be expressed as a set of points using the parametric equation PL + mDL for all values of m, where PL is a point on the line, and DL is the direction of the line. This is typically a unit vector - although in the demo below PL is not unit. The diagram below shows an infinite red line, which is defined by the green point on the line (PL) and the purple direction of the line (DL). The orange dot represents a point on the line for a specific value of m.

**Solution:**

```c++
inline double ClosestDistancePointToLine(
    const Vector3& point,      // PG
    const Vector3& linePoint,  // PL
    const Vector3& lineDir)    // DL
{
    const double lenSq = lineDir.LengthSquared();
    if (lenSq == 0.0)
        throw std::invalid_argument("Line direction cannot be zero vector");

    // Step 1
    Vector3 v = point - linePoint;

    // Step 2 (projection)
    double t = v.Dot(lineDir) / lenSq;
    Vector3 projection = lineDir * t;

    // Step 3
    Vector3 perpendicular = v - projection;

    // Step 4
    return perpendicular.Length();
}
```

**Test data:**
```c++

TEST(PointLineDistance, ClosestPointOnLine)
{
    double d = ClosestDistancePointToLine({ 2,3,4 }, { 0,0,0 }, { 1,1,1 });
    EXPECT_NEAR(d, 1.41, 0.01);
}

TEST(PointLineDistance, PointOnLine)
{
    double d = ClosestDistancePointToLine({ 3,6,9 }, { 0,0,0 }, { 1,2,3 });
    EXPECT_NEAR(d, 0.0, 0.0001);
}

TEST(PointLineDistance, VerticalLineCase)
{
    double d = ClosestDistancePointToLine({ 4,5,3 }, { 2,2,0 }, { 0,0,1 });
    EXPECT_NEAR(d, 3.61, 0.01);
}

TEST(PointLineDistance, HorizontalLineCase)
{
    double d = ClosestDistancePointToLine({ 3,4,5 }, { 0,0,0 }, { 1,0,0 });
    EXPECT_NEAR(d, 6.40, 0.01);
}

TEST(PointLineDistance, DiagonalLineCase)
{
    double d = ClosestDistancePointToLine({ 2,5,3 }, { 1,1,1 }, { 1,-1,1 });
    EXPECT_NEAR(d, 4.55, 0.01);
}
```
**Sample output:**

[----------] Global test environment set-up.
[----------] 5 tests from PointLineDistance
[ RUN      ] PointLineDistance.ClosestPointOnLine
[       OK ] PointLineDistance.ClosestPointOnLine (0 ms)
[ RUN      ] PointLineDistance.PointOnLine
[       OK ] PointLineDistance.PointOnLine (0 ms)
[ RUN      ] PointLineDistance.VerticalLineCase
[       OK ] PointLineDistance.VerticalLineCase (0 ms)
[ RUN      ] PointLineDistance.HorizontalLineCase
[       OK ] PointLineDistance.HorizontalLineCase (0 ms)
[ RUN      ] PointLineDistance.DiagonalLineCase
[       OK ] PointLineDistance.DiagonalLineCase (0 ms)
[----------] 5 tests from PointLineDistance (2 ms total)

**Reflection:**

In this task I used the distance function from the previous question to detect collisions between a sphere and an infinite line. If the shortest distance from the sphere centre to the line is less than or equal to the radius, the line intersects the sphere.

**Questions:**

No

### Q3. Sphere Line Intersection Test 

Now that you have the closest distance to a point on a line you can create a Sphere Line intersection test. All you need to do is to find the shortest distance from the point at the centre of the sphere to the line, and then compare that distance to the radius of the sphere.

**Solution:**

```c++

bool Sphere::IntersectsInfiniteLine(const Vector3& linePoint, const Vector3& lineDir) const
{
    return ClosestDistancePointToLine(this->GetPosition(), linePoint, lineDir) <= this->GetRadius();
}

```

**Test data:**
```c++


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

```
**Sample output:**

[----------] 6 tests from SphereLineIntersection
[ RUN      ] SphereLineIntersection.NoIntersectionCentreAtOrigin
[       OK ] SphereLineIntersection.NoIntersectionCentreAtOrigin (0 ms)
[ RUN      ] SphereLineIntersection.PassesThroughSphereCentreAtOrigin
[       OK ] SphereLineIntersection.PassesThroughSphereCentreAtOrigin (0 ms)
[ RUN      ] SphereLineIntersection.LineStartsInsideSphere
[       OK ] SphereLineIntersection.LineStartsInsideSphere (0 ms)
[ RUN      ] SphereLineIntersection.PassesThroughSphereCenter
[       OK ] SphereLineIntersection.PassesThroughSphereCenter (0 ms)
[ RUN      ] SphereLineIntersection.TangentLineCountsAsIntersect
[       OK ] SphereLineIntersection.TangentLineCountsAsIntersect (0 ms)
[ RUN      ] SphereLineIntersection.ZeroDirectionThrows
[       OK ] SphereLineIntersection.ZeroDirectionThrows (2 ms)
[----------] 6 tests from SphereLineIntersection (3 ms total)

**Reflection:**

In this task I calculated the shortest distance between a point and an infinite line using vector projection. The key idea is that the shortest distance must occur at a right angle to the line. By subtracting the line point from the general point, I obtained a vector representing their spatial separation. I then projected this vector onto the line direction using the dot product.

The projection gives the part of the vector that lies along the line. Subtracting this from the original vector leaves only the perpendicular component, and the length of this perpendicular vector is the shortest distance.

One important detail I learned is that the line direction does not need to be a unit vector. Dividing by the squared magnitude of the direction vector correctly scales the projection, which makes the function more flexible and avoids unnecessary normalisation operations.

**Questions:**

No

### Q4. Closest Distance from a Point to a Plane

You can define a plane as a point on the plane and the normal vector; the unit vector that is perpendicular to the plane. It can also be useful to store two other vectors that are on the plane.

**Solution:**

```c++
inline double ClosestDistancePointToPlane(
    const Vector3& point,       // PG
    const Vector3& planePoint,  // PP
    const Vector3& planeNormal) // N
{
    const double nLenSq = planeNormal.LengthSquared();
    if (nLenSq == 0.0)
        throw std::invalid_argument("Plane normal cannot be zero vector");

    const Vector3 v = point - planePoint;

    const double numerator = std::abs(v.Dot(planeNormal));
    const double denom = std::sqrt(nLenSq); // |N|

    return numerator / denom;
}
```

**Test data:**
```c++
#include "pch.h"
#include "MathUtils.h"

TEST(PointPlaneDistance, PointAbovePlane)
{
    double d = ClosestDistancePointToPlane({ 2,3,5 }, { 0,0,0 }, { 0,0,1 });
    EXPECT_NEAR(d, 5.00, 0.01);
}

TEST(PointPlaneDistance, PointBelowPlane)
{
    double d = ClosestDistancePointToPlane({ 2,3,-4 }, { 0,0,0 }, { 0,0,1 });
    EXPECT_NEAR(d, 4.00, 0.01);
}

TEST(PointPlaneDistance, PointOnPlane)
{
    // Plane: point (1,1,1), normal (1,1,1)
    // Point (0,2,1) lies in plane because (PG-PP)·N = 0
    double d = ClosestDistancePointToPlane({ 0,2,1 }, { 1,1,1 }, { 1,1,1 });
    EXPECT_NEAR(d, 0.00, 0.01);
}

TEST(PointPlaneDistance, PointCloseToPlane)
{
    double d = ClosestDistancePointToPlane({ 1,1,1 }, { 0,0,0 }, { 1,1,0 });
    EXPECT_NEAR(d, 1.41, 0.01);
}

TEST(PointPlaneDistance, PointWithNegativeCoordinates)
{
    double d = ClosestDistancePointToPlane({ -1,-1,-1 }, { -2,-2,-2 }, { 1,1,1 });
    EXPECT_NEAR(d, 1.73, 0.01);
}

TEST(PointPlaneDistance, PointAlongNormalVectorDirection)
{
    double d = ClosestDistancePointToPlane({ 1,1,0 }, { 0,0,0 }, { 1,1,0 });
    EXPECT_NEAR(d, 1.41, 0.01);
}

TEST(PointPlaneDistance, PointNearPlaneRandomDirection)
{
    double d = ClosestDistancePointToPlane({ 1,2,3 }, { 0,0,0 }, { 1,-1,0 });
    EXPECT_NEAR(d, 0.71, 0.01);
}

TEST(PointPlaneDistance, ZeroNormalThrows)
{
    EXPECT_THROW(ClosestDistancePointToPlane({ 1,2,3 }, { 0,0,0 }, { 0,0,0 }), std::invalid_argument);
}

```
**Sample output:**
[----------] 8 tests from PointPlaneDistance
[ RUN      ] PointPlaneDistance.PointAbovePlane
[       OK ] PointPlaneDistance.PointAbovePlane (0 ms)
[ RUN      ] PointPlaneDistance.PointBelowPlane
[       OK ] PointPlaneDistance.PointBelowPlane (0 ms)
[ RUN      ] PointPlaneDistance.PointOnPlane
[       OK ] PointPlaneDistance.PointOnPlane (0 ms)
[ RUN      ] PointPlaneDistance.PointCloseToPlane
[       OK ] PointPlaneDistance.PointCloseToPlane (0 ms)
[ RUN      ] PointPlaneDistance.PointWithNegativeCoordinates
[       OK ] PointPlaneDistance.PointWithNegativeCoordinates (0 ms)
[ RUN      ] PointPlaneDistance.PointAlongNormalVectorDirection
[       OK ] PointPlaneDistance.PointAlongNormalVectorDirection (0 ms)
[ RUN      ] PointPlaneDistance.PointNearPlaneRandomDirection
[       OK ] PointPlaneDistance.PointNearPlaneRandomDirection (0 ms)
[ RUN      ] PointPlaneDistance.ZeroNormalThrows
[       OK ] PointPlaneDistance.ZeroNormalThrows (1 ms)
[----------] 8 tests from PointPlaneDistance (3 ms total)

**Reflection:**
In this task I calculated the shortest distance from a general point to a plane using the plane’s normal vector. A plane can be defined by a point on the plane and a normal vector (perpendicular to the plane). The key idea is that the shortest distance to the plane is always measured in the direction of the normal.

To compute this, I formed the vector from the plane point to the general point and projected that vector onto the normal using the dot product. This gives a signed distance (positive on one side of the plane and negative on the other). Taking the absolute value gives the closest distance regardless of which side of the plane the point lies on.

One important detail was handling normals that are not unit length. If the normal is not normalised, the dot product result is scaled, so dividing by the magnitude of the normal is required to get the correct distance.

**Questions:**

No

### Q5. Sphere to Plane Collision

 Sphere to Plane Collision

**Solution:**

```c++
bool Plane::Intersects(const Sphere& sphere) const
{
    double d = ClosestDistancePointToPlane(sphere.GetPosition(), _Position, _Normal);
    return d <= sphere.GetRadius();
}
```

**Test data:**
```c++
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
```
**Sample output:**
[----------] 5 tests from SpherePlaneIntersection
[ RUN      ] SpherePlaneIntersection.AbovePlaneNoIntersection
[       OK ] SpherePlaneIntersection.AbovePlaneNoIntersection (0 ms)
[ RUN      ] SpherePlaneIntersection.TouchingPlane
[       OK ] SpherePlaneIntersection.TouchingPlane (0 ms)
[ RUN      ] SpherePlaneIntersection.CrossingPlane
[       OK ] SpherePlaneIntersection.CrossingPlane (0 ms)
[ RUN      ] SpherePlaneIntersection.SphereCentreOnPlane
[       OK ] SpherePlaneIntersection.SphereCentreOnPlane (0 ms)
[ RUN      ] SpherePlaneIntersection.NegativeSideStillIntersects
[       OK ] SpherePlaneIntersection.NegativeSideStillIntersects (0 ms)
[----------] 5 tests from SpherePlaneIntersection (1 ms total)

**Reflection:**
In this task I implemented sphere–plane collision detection by reusing the point-to-plane distance from Q4. The sphere intersects the plane if the closest distance from the sphere centre to the plane is less than or equal to the sphere radius. This is the same pattern as sphere–line collision: first calculate the minimum distance, then compare it to the radius.

This task reinforced the idea that collision detection can often be simplified into distance checks once the correct geometric distance function exists. It also showed the benefit of keeping the “distance to plane” calculation separate and reusable, because the collision test itself became very small and easy to reason about.

**Questions:**

No

