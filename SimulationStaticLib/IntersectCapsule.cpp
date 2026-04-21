#include "pch.h"
#include "Intersect.h"

#include "MathUtils.h"

#include <glm/glm.hpp>
#include <cmath>
#include <algorithm>
#include <limits>

CollisionManifold Intersect(const WorldCapsule& a, const WorldCapsule& b)
{
    CollisionManifold m{};

    glm::vec3 ca{}, cb{};
    ClosestPointsBetweenSegments(a.a, a.b, b.a, b.b, ca, cb);

    const glm::vec3 delta = cb - ca; // from A segment to B segment
    const float distSq = glm::dot(delta, delta);
    const float rSum = a.radius + b.radius;

    if (distSq > rSum * rSum)
        return m;

    const bool degenerate = (distSq <= kEps * kEps);

    const float dist = degenerate ? 0.0f : std::sqrt(distSq);
    const glm::vec3 n = degenerate ? glm::vec3(1, 0, 0) : (delta / dist);

    m.hit = true;
    m.normal = n;
    m.penetration = rSum - dist;

    // Contact point (midpoint between the two surface points, best-effort)
    const glm::vec3 pA = ca + n * a.radius;
    const glm::vec3 pB = cb - n * b.radius;
    m.contactPoint = 0.5f * (pA + pB);

    return m;
}

CollisionManifold Intersect(const WorldCapsule& c, const WorldPlane& p)
{
    CollisionManifold m{};

    // Closest point on capsule segment to the plane is one of the endpoints
    // only if segment is parallel, but in general: project endpoints and take minimal abs distance.
    // More robust: check both endpoints against plane distance and choose the closer one.
    const float dA = SignedDistanceToPlane(c.a, p.point, p.normal);
    const float dB = SignedDistanceToPlane(c.b, p.point, p.normal);

    // If segment crosses plane, closest point is along the segment where signed distance is 0
    glm::vec3 closestSegPoint{};
    float signedDClosest = 0.0f;

    if ((dA >= 0.0f && dB <= 0.0f) || (dA <= 0.0f && dB >= 0.0f))
    {
        // Interpolate t where distance is zero (avoid divide by 0)
        const float denom = (dA - dB);
        const float t = (std::abs(denom) > kEps) ? (dA / (dA - dB)) : 0.0f;
        closestSegPoint = c.a + (c.b - c.a) * t;
        signedDClosest = 0.0f;
    }
    else
    {
        // Take nearer endpoint
        if (std::abs(dA) < std::abs(dB))
        {
            closestSegPoint = c.a;
            signedDClosest = dA;
        }
        else
        {
            closestSegPoint = c.b;
            signedDClosest = dB;
        }
    }

    const float absD = std::abs(signedDClosest);
    if (absD > c.radius)
        return m;

    m.hit = true;

    // Normal points from plane toward capsule center-side
    const glm::vec3 n = (signedDClosest >= 0.0f) ? p.normal : -p.normal;

    m.normal = n;
    m.penetration = c.radius - absD;

    // Contact point on plane directly under the closest segment point
    m.contactPoint = closestSegPoint - p.normal * signedDClosest;

    return m;
}

CollisionManifold Intersect(const WorldCapsule& c, const WorldOBB& b)
{
    CollisionManifold m{};

    // Candidate points on capsule segment
    const glm::vec3 segMid = 0.5f * (c.a + c.b);

    glm::vec3 candidates[3] = { c.a, c.b, segMid };

    float bestDistSq = std::numeric_limits<float>::infinity();
    glm::vec3 bestSegPoint{ 0.0f };
    glm::vec3 bestBoxPoint{ 0.0f };

    for (const glm::vec3& pSeg : candidates)
    {
        const glm::vec3 pBox = ClosestPointOnOBB(
            pSeg,
            b.center, b.axisX, b.axisY, b.axisZ,
            b.halfExtents);

        const glm::vec3 d = pSeg - pBox;
        const float dsq = glm::dot(d, d);

        if (dsq < bestDistSq)
        {
            bestDistSq = dsq;
            bestSegPoint = pSeg;
            bestBoxPoint = pBox;
        }
    }

    // Also consider the closest point on the segment to the box center, then clamp that to OBB.
    {
        const glm::vec3 closestOnSegToCenter = ClosestPointOnSegment(b.center, c.a, c.b);
        const glm::vec3 closestOnBoxToThat = ClosestPointOnOBB(
            closestOnSegToCenter,
            b.center, b.axisX, b.axisY, b.axisZ,
            b.halfExtents);

        const glm::vec3 d = closestOnSegToCenter - closestOnBoxToThat;
        const float dsq = glm::dot(d, d);

        if (dsq < bestDistSq)
        {
            bestDistSq = dsq;
            bestSegPoint = closestOnSegToCenter;
            bestBoxPoint = closestOnBoxToThat;
        }
    }

    // Now treat capsule as sphere (radius c.radius) around bestSegPoint vs box
    const float r = c.radius;
    if (bestDistSq > r * r)
        return m;

    const float dist = std::sqrt(std::max(bestDistSq, kEps));
    glm::vec3 n = (dist > kEps) ? SafeNormalize(bestSegPoint - bestBoxPoint) : b.axisX;

    m.hit = true;
    m.normal = n;
    m.penetration = r - dist;

    // Contact point on box surface (best-effort)
    m.contactPoint = bestBoxPoint;

    return m;
}

CollisionManifold Intersect(const WorldCapsule& c, const WorldCylinder& y)
{
    CollisionManifold m{};

    glm::vec3 pc{}, py{};
    ClosestPointsBetweenSegments(c.a, c.b, y.a, y.b, pc, py);

    const glm::vec3 delta = py - pc; // from capsule axis to cylinder axis
    const float distSq = glm::dot(delta, delta);
    const float rSum = c.radius + y.radius;

    if (distSq > rSum * rSum)
        return m;

    const float dist = std::sqrt(std::max(distSq, kEps));
    const glm::vec3 n = (dist > kEps) ? (delta / dist) : glm::vec3(1, 0, 0);

    m.hit = true;
    m.normal = n;
    m.penetration = rSum - dist;

    // best-effort contact: midpoint between the two surface points
    const glm::vec3 pC = pc + n * c.radius;
    const glm::vec3 pY = py - n * y.radius;
    m.contactPoint = 0.5f * (pC + pY);

    return m;
}