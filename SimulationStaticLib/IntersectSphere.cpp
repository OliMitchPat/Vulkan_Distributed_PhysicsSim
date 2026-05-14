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

    // Closest point on the OBB to the sphere centre.
    const glm::vec3 closest = ClosestPointOnOBB(
        s.center,
        b.center, b.axisX, b.axisY, b.axisZ,
        b.halfExtents);

    const glm::vec3 deltaFromBoxToSphere = s.center - closest;
    const float distSq = glm::dot(deltaFromBoxToSphere, deltaFromBoxToSphere);

    // Work out whether the sphere centre is inside the box.
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
        // Normal outside case.
        // No hit if the closest point is farther away than the sphere radius.
        if (distSq > s.radius * s.radius)
            return m;

        const float dist = std::sqrt(std::max(distSq, kEps));

        // deltaFromBoxToSphere points from box -> sphere.
        // But this function is Sphere(A) vs OBB(B), so the solver wants A -> B.
        // Therefore normal must point from sphere -> box.
        const glm::vec3 n =
            (dist > kEps)
            ? -(deltaFromBoxToSphere / dist)
            : -b.axisY;

        m.hit = true;
        m.normal = n;
        m.penetration = s.radius - dist;

        // Contact point on the box surface.
        m.contactPoint = closest;

        return m;
    }

    // Sphere centre is inside the box.
    // Pick nearest face and push the sphere out through that face.
    const float mx = b.halfExtents.x - std::abs(x);
    const float my = b.halfExtents.y - std::abs(y);
    const float mz = b.halfExtents.z - std::abs(z);

    glm::vec3 outwardFaceNormal = (x >= 0.0f) ? b.axisX : -b.axisX;
    float faceMargin = mx;

    if (my < faceMargin)
    {
        faceMargin = my;
        outwardFaceNormal = (y >= 0.0f) ? b.axisY : -b.axisY;
    }

    if (mz < faceMargin)
    {
        faceMargin = mz;
        outwardFaceNormal = (z >= 0.0f) ? b.axisZ : -b.axisZ;
    }

    // outwardFaceNormal points box -> sphere/outside.
    // For Sphere(A) vs OBB(B), solver normal should point sphere -> box,
    // so use the opposite.
    const glm::vec3 n = -SafeNormalize(outwardFaceNormal, b.axisY);

    m.hit = true;
    m.normal = n;

    // Distance needed to move the sphere centre fully outside the chosen face.
    m.penetration = s.radius + faceMargin;

    // Build contact point on chosen face.
    glm::vec3 local{ x, y, z };

    if (outwardFaceNormal == b.axisX || outwardFaceNormal == -b.axisX)
    {
        local.x = (x >= 0.0f) ? b.halfExtents.x : -b.halfExtents.x;
    }
    else if (outwardFaceNormal == b.axisY || outwardFaceNormal == -b.axisY)
    {
        local.y = (y >= 0.0f) ? b.halfExtents.y : -b.halfExtents.y;
    }
    else
    {
        local.z = (z >= 0.0f) ? b.halfExtents.z : -b.halfExtents.z;
    }

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

    const glm::vec3 ab = c.b - c.a;
    const float heightSq = glm::dot(ab, ab);

    if (heightSq <= kEps)
        return m;

    const float height = std::sqrt(heightSq);
    const glm::vec3 axis = ab / height;

    // Position of sphere centre along the finite cylinder axis.
    // t = 0 at cap A, t = height at cap B.
    const float t = glm::dot(s.center - c.a, axis);

    const float clampedT = Clamp(t, 0.0f, height);
    const glm::vec3 axisPoint = c.a + axis * clampedT;

    const glm::vec3 radial = s.center - axisPoint;
    const float radialLenSq = glm::dot(radial, radial);
    const float radialLen = std::sqrt(std::max(radialLenSq, kEps));

    const glm::vec3 radialDir =
        (radialLen > kEps)
        ? radial / radialLen
        : glm::vec3(1.0f, 0.0f, 0.0f);

    // ------------------------------------------------------------
    // Case 1: sphere centre projects between the cylinder caps.
    // Collision with the curved side wall.
    // ------------------------------------------------------------
    if (t >= 0.0f && t <= height)
    {
        const float distanceFromCylinderSurface = radialLen - c.radius;

        if (distanceFromCylinderSurface > s.radius)
            return m;

        m.hit = true;

        // radialDir points cylinder -> sphere.
        // Solver wants Sphere(A) -> Cylinder(B), so flip it.
        m.normal = -radialDir;

        m.penetration = s.radius - distanceFromCylinderSurface;
        m.contactPoint = axisPoint + radialDir * c.radius;

        return m;
    }

    // ------------------------------------------------------------
    // Case 2: sphere is beyond one of the flat cylinder caps.
    // It may hit the cap face or the circular rim.
    // ------------------------------------------------------------
    const bool belowA = (t < 0.0f);
    const glm::vec3 capCenter = belowA ? c.a : c.b;

    // capOutward points cylinder -> sphere side of that cap.
    const glm::vec3 capOutward = belowA ? -axis : axis;

    const glm::vec3 toSphere = s.center - capCenter;
    const float signedCapDistance = glm::dot(toSphere, capOutward);

    const glm::vec3 radialOnCap = toSphere - capOutward * signedCapDistance;
    const float radialOnCapLenSq = glm::dot(radialOnCap, radialOnCap);
    const float radialOnCapLen = std::sqrt(std::max(radialOnCapLenSq, kEps));

    const glm::vec3 radialOnCapDir =
        (radialOnCapLen > kEps)
        ? radialOnCap / radialOnCapLen
        : glm::vec3(1.0f, 0.0f, 0.0f);

    if (radialOnCapLen <= c.radius)
    {
        // Sphere is over the flat cap disk.
        if (std::abs(signedCapDistance) > s.radius)
            return m;

        m.hit = true;

        // capOutward points cylinder -> sphere.
        // Solver wants sphere -> cylinder, so flip it.
        m.normal = -capOutward;

        m.penetration = s.radius - std::abs(signedCapDistance);
        m.contactPoint = s.center - capOutward * signedCapDistance;

        return m;
    }
    else
    {
        // Sphere is near the circular rim edge.
        const glm::vec3 closestRimPoint = capCenter + radialOnCapDir * c.radius;
        const glm::vec3 delta = s.center - closestRimPoint;

        const float distSq = glm::dot(delta, delta);

        if (distSq > s.radius * s.radius)
            return m;

        const float dist = std::sqrt(std::max(distSq, kEps));
        const glm::vec3 rimToSphere = delta / dist;

        m.hit = true;

        // rimToSphere points cylinder rim -> sphere.
        // Solver wants sphere -> cylinder rim, so flip it.
        m.normal = -rimToSphere;

        m.penetration = s.radius - dist;
        m.contactPoint = closestRimPoint;

        return m;
    }
}