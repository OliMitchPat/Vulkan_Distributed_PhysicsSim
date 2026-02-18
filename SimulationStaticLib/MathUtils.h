#pragma once
#include "Vector3.h"
#include <cmath>
#include <stdexcept>
#include "Sphere.h" 

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