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

    // Degenerate handling: if centers are the same (or extremely close),
    // choose a stable fallback normal and treat distance as 0.
    const bool degenerate = (distSq <= kEps * kEps);

    const float dist = degenerate ? 0.0f : std::sqrt(distSq);
    const glm::vec3 n = degenerate ? glm::vec3(1, 0, 0) : (delta / dist);

    m.hit = true;
    m.normal = n;
    m.penetration = rSum - dist;

    // Contact point: on surface of A toward B (fallback uses +X)
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

    // --- Compute closest point on OBB to sphere center (works for outside case) ---
    const glm::vec3 closest = ClosestPointOnOBB(
        s.center,
        b.center, b.axisX, b.axisY, b.axisZ,
        b.halfExtents);

    const glm::vec3 delta = s.center - closest;
    const float distSq = glm::dot(delta, delta);

    // --- Detect whether center is inside the OBB ---
    // Compute local coordinates in box space (signed distances along each axis)
    const glm::vec3 d = s.center - b.center;
    const float x = glm::dot(d, b.axisX);
    const float y = glm::dot(d, b.axisY);
    const float z = glm::dot(d, b.axisZ);

    const bool inside =
        (std::abs(x) <= b.halfExtents.x + kEps) &&
        (std::abs(y) <= b.halfExtents.y + kEps) &&
        (std::abs(z) <= b.halfExtents.z + kEps);

    if (!inside)
    {
        // Outside: classic sphere vs OBB distance test
        if (distSq > s.radius * s.radius)
            return m;

        const float dist = std::sqrt(std::max(distSq, kEps));
        const glm::vec3 n = (dist > kEps) ? (delta / dist) : b.axisX;

        m.hit = true;
        m.normal = n;
        m.penetration = s.radius - dist;
        m.contactPoint = closest; // point on box surface
        return m;
    }

    // --- Inside: choose nearest face to push OUT (stable manifold) ---
    // Margins to each face along each axis
    const float mx = b.halfExtents.x - std::abs(x);
    const float my = b.halfExtents.y - std::abs(y);
    const float mz = b.halfExtents.z - std::abs(z);

    // Pick smallest margin => nearest exit face
    glm::vec3 faceNormal = (x >= 0.0f) ? b.axisX : -b.axisX;
    float faceMargin = mx;

    if (my < faceMargin) { faceMargin = my; faceNormal = (y >= 0.0f) ? b.axisY : -b.axisY; }
    if (mz < faceMargin) { faceMargin = mz; faceNormal = (z >= 0.0f) ? b.axisZ : -b.axisZ; }

    // faceNormal points from box center toward the nearest face (and toward the sphere center side)
    // For sphere-vs-box manifold, normal should point from box surface toward sphere center.
    const glm::vec3 n = SafeNormalize(faceNormal, b.axisX);

    // Penetration: distance you must move sphere along -n to put it just outside the face
    // If center is faceMargin away from the face plane, sphere overlaps by (radius - faceMargin),
    // but for an "inside" recovery we want a positive push-out even when fully inside:
    m.hit = true;
    m.normal = n;
    m.penetration = s.radius + faceMargin;   // always >= radius (positive)

    // Contact point on the chosen face (best-effort): project to that face plane
    // Build the local point on that face: clamp the other two coordinates, set chosen axis to +/- extent
    glm::vec3 local{ x, y, z };
    if (n == b.axisX || n == -b.axisX) local.x = (x >= 0.0f) ? b.halfExtents.x : -b.halfExtents.x;
    else if (n == b.axisY || n == -b.axisY) local.y = (y >= 0.0f) ? b.halfExtents.y : -b.halfExtents.y;
    else local.z = (z >= 0.0f) ? b.halfExtents.z : -b.halfExtents.z;

    // Convert local coords back to world point on face
    m.contactPoint =
        b.center +
        b.axisX * local.x +
        b.axisY * local.y +
        b.axisZ * local.z;

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