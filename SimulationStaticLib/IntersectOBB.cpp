#include "pch.h"
#include "Intersect.h"

#include "MathUtils.h"

#include <glm/glm.hpp>
#include <cmath>
#include <algorithm>
#include <limits>

// Projects OBB half-extents onto a plane normal (world).
// Given unit axes and half extents, the radius of projection is sum(|dot(n, axis_i)|*e_i)
static float ProjectOBBRadiusOntoNormal(const WorldOBB& b, const glm::vec3& nUnit)
{
    return std::abs(glm::dot(nUnit, b.axisX)) * b.halfExtents.x +
        std::abs(glm::dot(nUnit, b.axisY)) * b.halfExtents.y +
        std::abs(glm::dot(nUnit, b.axisZ)) * b.halfExtents.z;
}

CollisionManifold Intersect(const WorldOBB& b, const WorldPlane& p)
{
    CollisionManifold m{};

    // Signed distance from box center to plane
    const float signedD = SignedDistanceToPlane(b.center, p.point, p.normal);
    const float r = ProjectOBBRadiusOntoNormal(b, p.normal);

    // If the interval [d - r, d + r] doesn't cross 0 => separated
    if (std::abs(signedD) > r)
        return m;

    m.hit = true;

    // Normal from OBB -> plane or plane -> OBB? We'll keep convention normal points from plane toward box center.
    // For consistency in response later, we define:
    // normal points from plane toward box (i.e., in direction that would push the box out of the plane).
    m.normal = (signedD >= 0.0f) ? p.normal : -p.normal;

    // Penetration = overlap amount along plane normal
    m.penetration = r - std::abs(signedD);

    // Best-effort contact point: project center onto plane, then move toward box by clamped amount.
    // (Good enough for now; later SAT contact generation can refine.)
    const glm::vec3 closestOnPlane = b.center - p.normal * signedD;
    m.contactPoint = closestOnPlane;

    return m;
}

// --- SAT helpers ---

static float AbsDot(const glm::vec3& a, const glm::vec3& b)
{
    return std::abs(glm::dot(a, b));
}

static void AddCandidateAxis(const glm::vec3& axis, glm::vec3& outAxis, float& outPenetration,
    const WorldOBB& a, const WorldOBB& b, const glm::vec3& delta)
{
    // Skip near-zero axis (parallel cross products)
    const float lenSq = glm::dot(axis, axis);
    if (lenSq <= kEps * kEps)
        return;

    const glm::vec3 n = axis * (1.0f / std::sqrt(lenSq)); // normalize

    // Project each OBB onto axis n:
    const float ra =
        AbsDot(n, a.axisX) * a.halfExtents.x +
        AbsDot(n, a.axisY) * a.halfExtents.y +
        AbsDot(n, a.axisZ) * a.halfExtents.z;

    const float rb =
        AbsDot(n, b.axisX) * b.halfExtents.x +
        AbsDot(n, b.axisY) * b.halfExtents.y +
        AbsDot(n, b.axisZ) * b.halfExtents.z;

    const float dist = std::abs(glm::dot(delta, n));
    const float overlap = (ra + rb) - dist;

    if (overlap < 0.0f)
    {
        // separating axis exists - caller must handle early out,
        // but we can't early out from here. We'll mark by setting penetration negative.
        outPenetration = -1.0f;
        return;
    }

    // Choose smallest overlap => best axis
    if (overlap < outPenetration)
    {
        // Ensure axis points from A -> B
        outAxis = (glm::dot(delta, n) >= 0.0f) ? n : -n;
        outPenetration = overlap;
    }
}

CollisionManifold Intersect(const WorldOBB& a, const WorldOBB& b)
{
    CollisionManifold m{};

    const glm::vec3 delta = b.center - a.center;

    // Candidate axes: 3 from A, 3 from B, and 9 cross products
    glm::vec3 bestAxis{ 1, 0, 0 };
    float bestPen = std::numeric_limits<float>::infinity();

    // Test function that can early out:
    auto TestAxis = [&](const glm::vec3& axis)
        {
            if (bestPen < 0.0f) return; // already separated

            const float lenSq = glm::dot(axis, axis);
            if (lenSq <= kEps * kEps) return;

            const glm::vec3 n = axis * (1.0f / std::sqrt(lenSq));

            const float ra =
                AbsDot(n, a.axisX) * a.halfExtents.x +
                AbsDot(n, a.axisY) * a.halfExtents.y +
                AbsDot(n, a.axisZ) * a.halfExtents.z;

            const float rb =
                AbsDot(n, b.axisX) * b.halfExtents.x +
                AbsDot(n, b.axisY) * b.halfExtents.y +
                AbsDot(n, b.axisZ) * b.halfExtents.z;

            const float dist = std::abs(glm::dot(delta, n));
            const float overlap = (ra + rb) - dist;

            if (overlap < 0.0f)
            {
                bestPen = -1.0f; // separated
                return;
            }

            if (overlap < bestPen)
            {
                bestPen = overlap;
                bestAxis = (glm::dot(delta, n) >= 0.0f) ? n : -n;
            }
        };

    // 3 face normals of each box
    TestAxis(a.axisX); TestAxis(a.axisY); TestAxis(a.axisZ);
    TestAxis(b.axisX); TestAxis(b.axisY); TestAxis(b.axisZ);

    // 9 cross products
    TestAxis(glm::cross(a.axisX, b.axisX));
    TestAxis(glm::cross(a.axisX, b.axisY));
    TestAxis(glm::cross(a.axisX, b.axisZ));

    TestAxis(glm::cross(a.axisY, b.axisX));
    TestAxis(glm::cross(a.axisY, b.axisY));
    TestAxis(glm::cross(a.axisY, b.axisZ));

    TestAxis(glm::cross(a.axisZ, b.axisX));
    TestAxis(glm::cross(a.axisZ, b.axisY));
    TestAxis(glm::cross(a.axisZ, b.axisZ));

    if (bestPen < 0.0f || !std::isfinite(bestPen))
        return m;

    m.hit = true;
    m.normal = bestAxis;
    m.penetration = bestPen;

    // Contact point generation for OBB-OBB is complex.
    // For now: use a simple best-effort contact point near the middle of overlap.
    // This is sufficient for “detection” marks and even basic impulse response.
    m.contactPoint = 0.5f * (a.center + b.center) - bestAxis * (0.5f * bestPen);

    return m;
}