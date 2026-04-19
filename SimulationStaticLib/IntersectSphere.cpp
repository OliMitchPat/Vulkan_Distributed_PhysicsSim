#include "pch.h"
#include "Intersect.h"

#include "MathUtils.h"

#include <glm/glm.hpp>
#include <cmath>

CollisionManifold Intersect(const WorldSphere& a, const WorldSphere& b)
{
    CollisionManifold m{};

    const glm::vec3 delta = b.center - a.center;
    const float distSq = glm::dot(delta, delta);
    const float rSum = a.radius + b.radius;

    if (distSq > rSum * rSum)
        return m;

    const float dist = std::sqrt(std::max(distSq, kEps));
    const glm::vec3 n = (dist > kEps) ? (delta / dist) : glm::vec3(1, 0, 0);

    m.hit = true;
    m.normal = n;
    m.penetration = rSum - dist;

    // Contact point: on surface of A toward B
    m.contactPoint = a.center + n * a.radius;
    return m;
}

CollisionManifold Intersect(const WorldSphere& s, const WorldPlane& p)
{
    CollisionManifold m{};

    // Signed distance from sphere center to plane
    const float signedD = SignedDistanceToPlane(s.center, p.point, p.normal);
    const float absD = std::abs(signedD);

    if (absD > s.radius)
        return m;

    m.hit = true;

    // Normal should point from plane toward sphere (A->B convention is ambiguous here).
    // We'll define: normal points from plane into the half-space where the sphere center lies.
    const glm::vec3 n = (signedD >= 0.0f) ? p.normal : -p.normal;

    m.normal = n;
    m.penetration = s.radius - absD;

    // Closest point on plane to sphere center
    const glm::vec3 closest = s.center - p.normal * signedD;
    m.contactPoint = closest;
    return m;
}

// Helper: closest point on OBB to a point (world OBB)
static glm::vec3 ClosestPointOnOBB(const glm::vec3& p, const WorldOBB& b)
{
    // Vector from box center to point
    glm::vec3 d = p - b.center;

    glm::vec3 q = b.center;

    // Project d onto each axis, clamp to extents
    float dist = glm::dot(d, b.axisX);
    dist = Clamp(dist, -b.halfExtents.x, b.halfExtents.x);
    q += b.axisX * dist;

    dist = glm::dot(d, b.axisY);
    dist = Clamp(dist, -b.halfExtents.y, b.halfExtents.y);
    q += b.axisY * dist;

    dist = glm::dot(d, b.axisZ);
    dist = Clamp(dist, -b.halfExtents.z, b.halfExtents.z);
    q += b.axisZ * dist;

    return q;
}

CollisionManifold Intersect(const WorldSphere& s, const WorldOBB& b)
{
    CollisionManifold m{};

    const glm::vec3 closest = ClosestPointOnOBB(s.center, b);
    const glm::vec3 delta = s.center - closest;
    const float distSq = glm::dot(delta, delta);

    if (distSq > s.radius * s.radius)
        return m;

    const float dist = std::sqrt(std::max(distSq, kEps));
    glm::vec3 n = (dist > kEps) ? (delta / dist) : b.axisX; // fallback

    m.hit = true;
    m.normal = n;
    m.penetration = s.radius - dist;
    m.contactPoint = closest;
    return m;
}

CollisionManifold Intersect(const WorldSphere& s, const WorldCapsule& c)
{
    CollisionManifold m{};

    const glm::vec3 closest = ClosestPointOnSegment(s.center, c.a, c.b);
    const glm::vec3 delta = s.center - closest;

    const float rSum = s.radius + c.radius;
    const float distSq = glm::dot(delta, delta);

    if (distSq > rSum * rSum)
        return m;

    const float dist = std::sqrt(std::max(distSq, kEps));
    const glm::vec3 n = (dist > kEps) ? (delta / dist) : glm::vec3(1, 0, 0);

    m.hit = true;
    m.normal = n;
    m.penetration = rSum - dist;

    // Contact point on capsule surface toward sphere (best-effort)
    m.contactPoint = closest + n * c.radius;
    return m;
}

CollisionManifold Intersect(const WorldSphere& s, const WorldCylinder& c)
{
    CollisionManifold m{};

    // Approx narrow phase:
    // - Find closest point on cylinder axis segment to sphere center
    // - Check radial distance <= (c.radius + s.radius)
    // - Also ensure closest point lies within finite height via segment clamp (done by ClosestPointOnSegment)
    const glm::vec3 closestAxis = ClosestPointOnSegment(s.center, c.a, c.b);
    const glm::vec3 delta = s.center - closestAxis;

    const float rSum = s.radius + c.radius;
    const float distSq = glm::dot(delta, delta);

    if (distSq > rSum * rSum)
        return m;

    const float dist = std::sqrt(std::max(distSq, kEps));
    const glm::vec3 n = (dist > kEps) ? (delta / dist) : glm::vec3(1, 0, 0);

    m.hit = true;
    m.normal = n;
    m.penetration = rSum - dist;

    // Contact point on cylinder surface (best-effort)
    m.contactPoint = closestAxis + n * c.radius;
    return m;
}