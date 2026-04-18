#include "pch.h"
#include "Plane.h"
#include <stdexcept>
#include "Sphere.h"
#include <cmath>
#include "MathUtils.h"

Plane::Plane(const glm::vec3& pointOnPlane, const glm::vec3& normal)
    : Collider(pointOnPlane)
{
    const float lenSq = glm::dot(normal, normal);
    if (lenSq == 0.0f)
        throw std::invalid_argument("Plane normal must be non-zero");

    // Normalize once so future dot/distance computations are consistent
    m_normal = glm::normalize(normal);
}

// Convention:
// "Inside" means on the plane or on the side the normal points to.
// i.e. dot(point - planePoint, normal) >= 0
bool Plane::IsInside(const glm::vec3& point) const
{
    const glm::vec3 v = point - m_position;
    return glm::dot(v, m_normal) >= 0.0f;
}

bool Plane::Intersects(const Line& line) const
{
    // Segment-plane intersection:
    // signed distances d0 = dot(S - P0, N), d1 = dot(E - P0, N)
    const float d0 = glm::dot(line.Start - m_position, m_normal);
    const float d1 = glm::dot(line.End - m_position, m_normal);

    // If both are exactly 0, the segment lies in plane
    if (d0 == 0.0f && d1 == 0.0f)
        return true;

    // Intersects if either endpoint is on plane or if signs differ
    return (d0 == 0.0f) || (d1 == 0.0f) || (d0 < 0.0f && d1 > 0.0f) || (d0 > 0.0f && d1 < 0.0f);
}

bool Plane::Intersects(const Sphere& sphere) const
{
    // Since m_normal is normalized, point-to-plane distance becomes abs(dot(p - p0, n))
    const float d = std::abs(glm::dot(sphere.GetPosition() - m_position, m_normal));
    return d <= sphere.GetRadius();
}