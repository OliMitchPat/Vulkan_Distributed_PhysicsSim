#pragma once
#include <glm/glm.hpp>
#include <cmath>
#include <stdexcept>

// Distance from point to infinite line defined by linePoint + t*lineDir
inline float ClosestDistancePointToLine(
    const glm::vec3& point,     // PG
    const glm::vec3& linePoint, // PL
    const glm::vec3& lineDir)   // DL
{
    const float lenSq = glm::dot(lineDir, lineDir); // |DL|^2
    if (lenSq == 0.0f)
        throw std::invalid_argument("Line direction cannot be zero vector");

    // v = PG - PL
    const glm::vec3 v = point - linePoint;

    // t = (v·DL)/|DL|^2
    const float t = glm::dot(v, lineDir) / lenSq;

    // perpendicular = v - DL*t
    const glm::vec3 perpendicular = v - (lineDir * t);

    return glm::length(perpendicular);
}

// Distance from point to plane defined by planePoint and planeNormal
inline float ClosestDistancePointToPlane(
    const glm::vec3& point,       // PG
    const glm::vec3& planePoint,  // PP
    const glm::vec3& planeNormal) // N
{
    const float nLenSq = glm::dot(planeNormal, planeNormal); // |N|^2
    if (nLenSq == 0.0f)
        throw std::invalid_argument("Plane normal cannot be zero vector");

    const glm::vec3 v = point - planePoint;

    const float numerator = std::abs(glm::dot(v, planeNormal));
    const float denom = std::sqrt(nLenSq); // |N|

    return numerator / denom;
}