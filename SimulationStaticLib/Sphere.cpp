#include "pch.h"
#include "Sphere.h"
#include <stdexcept>
#include <cmath>
#include "MathUtils.h"

Sphere::Sphere(const glm::vec3& centre, float radius)
    : Collider(centre), m_radius(radius)
{
    if (radius < 0.0f)
        throw std::invalid_argument("Sphere radius must be non-negative");
}

bool Sphere::IsInside(const glm::vec3& point) const
{
    const glm::vec3 diff = point - m_position;
    const float r2 = m_radius * m_radius;
    return glm::dot(diff, diff) <= r2;
}

bool Sphere::Intersects(const Line& line) const
{
    // Segment-sphere intersection:
    // Solve |(S + t*(E-S)) - C|^2 = r^2 for t in [0,1]
    const glm::vec3 d = line.End - line.Start;    // segment direction
    const glm::vec3 m = line.Start - m_position;  // from centre to start

    const float a = glm::dot(d, d);
    const float b = 2.0f * glm::dot(m, d);
    const float c = glm::dot(m, m) - (m_radius * m_radius);

    // Degenerate segment (Start == End): treat as point test
    if (a == 0.0f)
        return glm::dot(m, m) <= (m_radius * m_radius);

    const float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f)
        return false;

    const float sqrtDisc = std::sqrt(discriminant);
    const float inv2a = 1.0f / (2.0f * a);

    const float t0 = (-b - sqrtDisc) * inv2a;
    const float t1 = (-b + sqrtDisc) * inv2a;

    // intersects segment if either root is within [0,1]
    return (t0 >= 0.0f && t0 <= 1.0f) || (t1 >= 0.0f && t1 <= 1.0f);
}

bool Sphere::CollideWith(const Sphere& other) const
{
    const glm::vec3 delta = other.GetPosition() - m_position;
    const float rSum = m_radius + other.m_radius;

    return glm::dot(delta, delta) <= (rSum * rSum);
}

bool Sphere::IntersectsInfiniteLine(const glm::vec3& linePoint, const glm::vec3& lineDir) const
{
    return ClosestDistancePointToLine(GetPosition(), linePoint, lineDir) <= GetRadius();
}