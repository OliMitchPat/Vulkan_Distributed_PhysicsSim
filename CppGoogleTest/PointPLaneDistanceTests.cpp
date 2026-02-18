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
