#include "pch.h"
#include "Intersect.h"

#include "MathUtils.h"

#include <glm/glm.hpp>
#include <cmath>
#include <algorithm>
#include <limits>


CollisionManifold Intersect(const WorldCylinder& c, const WorldPlane& p)
{
    CollisionManifold m{};

    // Cylinder is axis segment AB with radius.
    // Compute signed distances of endpoints.
    const float dA = SignedDistanceToPlane(c.a, p.point, p.normal);
    const float dB = SignedDistanceToPlane(c.b, p.point, p.normal);

    // Closest point on axis segment to plane:
    glm::vec3 closestAxisPoint{};
    float signedDClosest = 0.0f;

    // If axis crosses plane, closest signed distance is 0 at crossing.
    if ((dA >= 0.0f && dB <= 0.0f) || (dA <= 0.0f && dB >= 0.0f))
    {
        const float denom = (dA - dB);
        const float t = (std::abs(denom) > kEps) ? (dA / (dA - dB)) : 0.0f;
        closestAxisPoint = c.a + (c.b - c.a) * t;
        signedDClosest = 0.0f;
    }
    else
    {
        // Otherwise choose endpoint closer to plane
        if (std::abs(dA) < std::abs(dB))
        {
            closestAxisPoint = c.a;
            signedDClosest = dA;
        }
        else
        {
            closestAxisPoint = c.b;
            signedDClosest = dB;
        }
    }

    const float absD = std::abs(signedDClosest);
    if (absD > c.radius)
        return m;

    m.hit = true;
    m.normal = (signedDClosest >= 0.0f) ? p.normal : -p.normal;
    m.penetration = c.radius - absD;

    // Contact point on plane under the closest axis point
    m.contactPoint = closestAxisPoint - p.normal * signedDClosest;
    return m;
}

CollisionManifold Intersect(const WorldCylinder& c, const WorldOBB& b)
{
    CollisionManifold m{};

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

    // Also check the closest point on the axis to the box center
    {
        const glm::vec3 closestOnAxis = ClosestPointOnSegment(b.center, c.a, c.b);
        const glm::vec3 closestOnBox = ClosestPointOnOBB(
            closestOnAxis,
            b.center, b.axisX, b.axisY, b.axisZ,
            b.halfExtents);

        const glm::vec3 d = closestOnAxis - closestOnBox;
        const float dsq = glm::dot(d, d);

        if (dsq < bestDistSq)
        {
            bestDistSq = dsq;
            bestSegPoint = closestOnAxis;
            bestBoxPoint = closestOnBox;
        }
    }

    const float r = c.radius;
    if (bestDistSq > r * r)
        return m;

    const float dist = std::sqrt(std::max(bestDistSq, kEps));
    glm::vec3 n = (dist > kEps) ? SafeNormalize(bestSegPoint - bestBoxPoint) : b.axisX;

    m.hit = true;
    m.normal = n;
    m.penetration = r - dist;

    // Best-effort contact point on box
    m.contactPoint = bestBoxPoint;
    return m;
}

CollisionManifold Intersect(const WorldCylinder& a, const WorldCylinder& b)
{
    CollisionManifold m{};

    glm::vec3 pa{}, pb{};
    ClosestPointsBetweenSegments(a.a, a.b, b.a, b.b, pa, pb);

    const glm::vec3 delta = pb - pa; // A axis -> B axis
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

    const glm::vec3 pA = pa + n * a.radius;
    const glm::vec3 pB = pb - n * b.radius;
    m.contactPoint = 0.5f * (pA + pB);

    return m;
}