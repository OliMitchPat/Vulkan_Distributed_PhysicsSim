#include "pch.h"
#include "MathUtils.h"

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

