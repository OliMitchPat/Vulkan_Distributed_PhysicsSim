#include "pch.h"
#include "Containment.h"

#include "MathUtils.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

// ------------------------ Helpers ------------------------

static inline float Length(const glm::vec3& v) { return std::sqrt(glm::dot(v, v)); }

static inline glm::vec3 SafeUnit(const glm::vec3& v, const glm::vec3& fallback = glm::vec3(1, 0, 0))
{
    const float lsq = glm::dot(v, v);
    if (lsq <= kEps * kEps) return fallback;
    return v * (1.0f / std::sqrt(lsq));
}

// Return max signed distance of a shape to a plane normal direction n (unit) relative to shape "center" distances.
// For plane containers we need: maxSignedDistance = max_{x in shape} dot(n, x) - dot(n, planePoint)
// but we can compute it as: signedD(center) + supportRadiusAlong(n)
static float SupportRadiusAlong_PlaneNormal(const WorldSphere& s, const glm::vec3& nUnit)
{
    (void)nUnit;
    return s.radius;
}

static float SupportRadiusAlong_PlaneNormal(const WorldCapsule& c, const glm::vec3& nUnit)
{
    // capsule = segment swept by sphere radius
    const float dA = glm::dot(nUnit, c.a);
    const float dB = glm::dot(nUnit, c.b);
    const float maxEnd = std::max(dA, dB);
    const float centerish = maxEnd; // we’ll use this pattern in plane contain directly
    (void)centerish;
    return c.radius; // end selection handled outside
}

static float ProjectOBBRadiusOntoNormal(const WorldOBB& b, const glm::vec3& nUnit)
{
    return std::abs(glm::dot(nUnit, b.axisX)) * b.halfExtents.x +
        std::abs(glm::dot(nUnit, b.axisY)) * b.halfExtents.y +
        std::abs(glm::dot(nUnit, b.axisZ)) * b.halfExtents.z;
}

// Compute OBB corners (8)
static void GetOBBCorners(const WorldOBB& b, glm::vec3 out[8])
{
    const glm::vec3 ex = b.axisX * b.halfExtents.x;
    const glm::vec3 ey = b.axisY * b.halfExtents.y;
    const glm::vec3 ez = b.axisZ * b.halfExtents.z;

    int i = 0;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2)
                out[i++] = b.center + float(sx) * ex + float(sy) * ey + float(sz) * ez;
}

// Sphere inside OBB (exact containment)
static CollisionManifold ContainSphereInOBB(const WorldSphere& a, const WorldOBB& c)
{
    CollisionManifold m{};

    const glm::vec3 d = a.center - c.center;
    const float x = glm::dot(d, c.axisX);
    const float y = glm::dot(d, c.axisY);
    const float z = glm::dot(d, c.axisZ);

    const float mx = c.halfExtents.x - std::abs(x);
    const float my = c.halfExtents.y - std::abs(y);
    const float mz = c.halfExtents.z - std::abs(z);

    if (mx >= a.radius && my >= a.radius && mz >= a.radius)
        return m; // fully inside

    // most violated face = smallest margin
    float minMargin = mx;
    glm::vec3 n = (x >= 0.0f) ? -c.axisX : c.axisX;

    if (my < minMargin) { minMargin = my; n = (y >= 0.0f) ? -c.axisY : c.axisY; }
    if (mz < minMargin) { minMargin = mz; n = (z >= 0.0f) ? -c.axisZ : c.axisZ; }

    m.hit = true;
    m.normal = n;
    m.penetration = a.radius - minMargin;

    // contact point on the violated face plane (best-effort)
    if (n == c.axisX || n == -c.axisX)
        m.contactPoint = c.center + ((x >= 0.0f) ? c.axisX : -c.axisX) * c.halfExtents.x;
    else if (n == c.axisY || n == -c.axisY)
        m.contactPoint = c.center + ((y >= 0.0f) ? c.axisY : -c.axisY) * c.halfExtents.y;
    else
        m.contactPoint = c.center + ((z >= 0.0f) ? c.axisZ : -c.axisZ) * c.halfExtents.z;

    return m;
}

// Sphere inside Cylinder (exact containment)
static CollisionManifold ContainSphereInCylinder(const WorldSphere& a, const WorldCylinder& c)
{
    CollisionManifold m{};

    const glm::vec3 ab = c.b - c.a;
    const float abLenSq = glm::dot(ab, ab);
    if (abLenSq <= kEps)
        return m;

    const float height = std::sqrt(abLenSq);
    const glm::vec3 axis = ab / height;

    const float t = glm::dot(a.center - c.a, axis);
    const float tClamped = Clamp(t, 0.0f, height);
    const glm::vec3 closestAxis = c.a + axis * tClamped;

    const glm::vec3 radial = a.center - closestAxis;
    const float rLen = std::sqrt(std::max(glm::dot(radial, radial), kEps));

    // Must satisfy: rLen <= (R - a.radius)
    const float sideMargin = (c.radius - a.radius) - rLen;

    // Must satisfy: a.radius <= t <= height - a.radius
    const float capMargin = std::min(t, height - t) - a.radius;

    if (sideMargin >= 0.0f && capMargin >= 0.0f)
        return m; // fully inside

    m.hit = true;

    // choose most violated
    if (sideMargin < capMargin)
    {
        // side wall violation
        const glm::vec3 outward = radial / rLen; // from axis to center
        m.normal = -outward; // push inward
        m.penetration = -sideMargin;
        m.contactPoint = closestAxis + outward * c.radius;
    }
    else
    {
        // cap violation (t below a.radius or above height-a.radius)
        if (t < height * 0.5f)
        {
            m.normal = axis; // push toward +axis
            m.penetration = -capMargin;
            m.contactPoint = c.a;
        }
        else
        {
            m.normal = -axis;
            m.penetration = -capMargin;
            m.contactPoint = c.b;
        }
    }

    return m;
}

// Sphere inside Sphere container (exact containment)
static CollisionManifold ContainSphereInSphere(const WorldSphere& a, const WorldSphere& c)
{
    CollisionManifold m{};

    const glm::vec3 delta = a.center - c.center;
    const float dist = Length(delta);
    const float allowed = c.radius - a.radius; // center must stay within this

    if (dist <= allowed)
        return m;

    const glm::vec3 outward = (dist > kEps) ? (delta / dist) : glm::vec3(1, 0, 0);

    m.hit = true;
    m.normal = -outward;               // push back toward container center
    m.penetration = dist - allowed;
    m.contactPoint = c.center + outward * c.radius; // point on container sphere surface
    return m;
}

// ------------------------ Plane container (half-space) ------------------------
// Inside convention: SignedDistanceToPlane(x) <= 0
// For a shape, compute its maximum signed distance; if maxD > 0 it's outside.
CollisionManifold Contain(const WorldSphere& a, const WorldPlane& p)
{
    CollisionManifold m{};
    const float centerD = SignedDistanceToPlane(a.center, p.point, p.normal);
    const float maxD = centerD + a.radius;

    if (maxD <= 0.0f)
        return m;

    m.hit = true;
    m.normal = -p.normal;     // push back inside (toward negative signed distance)
    m.penetration = maxD;
    m.contactPoint = a.center - p.normal * centerD;
    return m;
}

CollisionManifold Contain(const WorldCapsule& a, const WorldPlane& p)
{
    CollisionManifold m{};
    const float dA = SignedDistanceToPlane(a.a, p.point, p.normal);
    const float dB = SignedDistanceToPlane(a.b, p.point, p.normal);
    const float maxEnd = std::max(dA, dB);
    const float maxD = maxEnd + a.radius;

    if (maxD <= 0.0f)
        return m;

    m.hit = true;
    m.normal = -p.normal;
    m.penetration = maxD;
    m.contactPoint = (dA > dB) ? (a.a - p.normal * dA) : (a.b - p.normal * dB);
    return m;
}

CollisionManifold Contain(const WorldCylinder& a, const WorldPlane& p)
{
    CollisionManifold m{};
    const float dA = SignedDistanceToPlane(a.a, p.point, p.normal);
    const float dB = SignedDistanceToPlane(a.b, p.point, p.normal);
    const float maxEnd = std::max(dA, dB);
    const float maxD = maxEnd + a.radius;

    if (maxD <= 0.0f)
        return m;

    m.hit = true;
    m.normal = -p.normal;
    m.penetration = maxD;
    m.contactPoint = (dA > dB) ? (a.a - p.normal * dA) : (a.b - p.normal * dB);
    return m;
}

CollisionManifold Contain(const WorldOBB& a, const WorldPlane& p)
{
    CollisionManifold m{};
    const float centerD = SignedDistanceToPlane(a.center, p.point, p.normal);
    const float r = ProjectOBBRadiusOntoNormal(a, p.normal);
    const float maxD = centerD + r;

    if (maxD <= 0.0f)
        return m;

    m.hit = true;
    m.normal = -p.normal;
    m.penetration = maxD;
    m.contactPoint = a.center - p.normal * centerD;
    return m;
}

// ------------------------ Sphere container ------------------------
CollisionManifold Contain(const WorldSphere& a, const WorldSphere& c) { return ContainSphereInSphere(a, c); }
CollisionManifold Contain(const WorldCapsule& a, const WorldSphere& c)
{
    // Capsule inside sphere: the farthest point from center is max(dist(center->A), dist(center->B)) + radius
    CollisionManifold m{};
    const float dA = Length(a.a - c.center);
    const float dB = Length(a.b - c.center);
    const float maxD = std::max(dA, dB) + a.radius;
    if (maxD <= c.radius) return m;

    const glm::vec3 worst = (dA > dB) ? a.a : a.b;
    const glm::vec3 delta = worst - c.center;
    const float dist = Length(delta);
    const glm::vec3 outward = (dist > kEps) ? (delta / dist) : glm::vec3(1, 0, 0);

    m.hit = true;
    m.normal = -outward;
    m.penetration = maxD - c.radius;
    m.contactPoint = c.center + outward * c.radius;
    return m;
}
CollisionManifold Contain(const WorldCylinder& a, const WorldSphere& c)
{
    CollisionManifold m{};
    const float dA = Length(a.a - c.center);
    const float dB = Length(a.b - c.center);
    const float maxD = std::max(dA, dB) + a.radius;
    if (maxD <= c.radius) return m;

    const glm::vec3 worst = (dA > dB) ? a.a : a.b;
    const glm::vec3 delta = worst - c.center;
    const float dist = Length(delta);
    const glm::vec3 outward = (dist > kEps) ? (delta / dist) : glm::vec3(1, 0, 0);

    m.hit = true;
    m.normal = -outward;
    m.penetration = maxD - c.radius;
    m.contactPoint = c.center + outward * c.radius;
    return m;
}
CollisionManifold Contain(const WorldOBB& a, const WorldSphere& c)
{
    // OBB inside sphere: farthest corner from sphere center must be <= radius
    CollisionManifold m{};
    glm::vec3 corners[8]; GetOBBCorners(a, corners);

    float worst = -1.0f;
    glm::vec3 worstCorner{ 0.0f };
    for (int i = 0; i < 8; i++)
    {
        const float d = Length(corners[i] - c.center);
        if (d > worst) { worst = d; worstCorner = corners[i]; }
    }

    if (worst <= c.radius)
        return m;

    const glm::vec3 delta = worstCorner - c.center;
    const float dist = Length(delta);
    const glm::vec3 outward = (dist > kEps) ? (delta / dist) : glm::vec3(1, 0, 0);

    m.hit = true;
    m.normal = -outward;
    m.penetration = worst - c.radius;
    m.contactPoint = c.center + outward * c.radius;
    return m;
}

// ------------------------ OBB container ------------------------
CollisionManifold Contain(const WorldSphere& a, const WorldOBB& c) { return ContainSphereInOBB(a, c); }

CollisionManifold Contain(const WorldCapsule& a, const WorldOBB& c)
{
    // Proper-ish containment: both endpoint spheres must be inside OBB.
    // This is exact for face constraints; corner cases can still require edge constraints,
    // but for most scenes it behaves correctly.
    CollisionManifold mA = ContainSphereInOBB(WorldSphere{ a.a, a.radius }, c);
    CollisionManifold mB = ContainSphereInOBB(WorldSphere{ a.b, a.radius }, c);

    if (!mA.hit && !mB.hit) return {};

    // Return the worse violation
    if (mA.hit && (!mB.hit || mA.penetration >= mB.penetration)) return mA;
    return mB;
}

CollisionManifold Contain(const WorldCylinder& a, const WorldOBB& c)
{
    // Cylinder inside OBB: treat as two end spheres against faces (same reasoning as capsule)
    CollisionManifold mA = ContainSphereInOBB(WorldSphere{ a.a, a.radius }, c);
    CollisionManifold mB = ContainSphereInOBB(WorldSphere{ a.b, a.radius }, c);

    if (!mA.hit && !mB.hit) return {};
    if (mA.hit && (!mB.hit || mA.penetration >= mB.penetration)) return mA;
    return mB;
}

CollisionManifold Contain(const WorldOBB& a, const WorldOBB& c)
{
    // OBB inside OBB: all corners must be inside container (with zero margin).
    // Find worst violating corner, then push inward along the most violated face normal.
    CollisionManifold m{};

    glm::vec3 corners[8]; GetOBBCorners(a, corners);

    float worstPen = 0.0f;
    glm::vec3 bestNormal{ 0.0f };

    for (int i = 0; i < 8; i++)
    {
        const glm::vec3 d = corners[i] - c.center;
        const float x = glm::dot(d, c.axisX);
        const float y = glm::dot(d, c.axisY);
        const float z = glm::dot(d, c.axisZ);

        const float px = std::abs(x) - c.halfExtents.x;
        const float py = std::abs(y) - c.halfExtents.y;
        const float pz = std::abs(z) - c.halfExtents.z;

        // if any positive, corner is outside
        if (px > 0.0f || py > 0.0f || pz > 0.0f)
        {
            // choose axis with maximum violation for this corner
            if (px >= py && px >= pz)
            {
                const glm::vec3 n = (x >= 0.0f) ? -c.axisX : c.axisX;
                if (px > worstPen) { worstPen = px; bestNormal = n; }
            }
            else if (py >= px && py >= pz)
            {
                const glm::vec3 n = (y >= 0.0f) ? -c.axisY : c.axisY;
                if (py > worstPen) { worstPen = py; bestNormal = n; }
            }
            else
            {
                const glm::vec3 n = (z >= 0.0f) ? -c.axisZ : c.axisZ;
                if (pz > worstPen) { worstPen = pz; bestNormal = n; }
            }
        }
    }

    if (worstPen <= 0.0f)
        return m;

    m.hit = true;
    m.normal = bestNormal;
    m.penetration = worstPen;
    m.contactPoint = a.center; // best-effort
    return m;
}

// ------------------------ Cylinder container ------------------------
CollisionManifold Contain(const WorldSphere& a, const WorldCylinder& c) { return ContainSphereInCylinder(a, c); }

CollisionManifold Contain(const WorldCapsule& a, const WorldCylinder& c)
{
    // Capsule inside cylinder: endpoint spheres must be inside.
    // This is strong and works well; exact swept constraints are more complex.
    CollisionManifold mA = ContainSphereInCylinder(WorldSphere{ a.a, a.radius }, c);
    CollisionManifold mB = ContainSphereInCylinder(WorldSphere{ a.b, a.radius }, c);
    if (!mA.hit && !mB.hit) return {};
    if (mA.hit && (!mB.hit || mA.penetration >= mB.penetration)) return mA;
    return mB;
}

CollisionManifold Contain(const WorldCylinder& a, const WorldCylinder& c)
{
    // Cylinder inside cylinder: same endpoint-sphere method
    CollisionManifold mA = ContainSphereInCylinder(WorldSphere{ a.a, a.radius }, c);
    CollisionManifold mB = ContainSphereInCylinder(WorldSphere{ a.b, a.radius }, c);
    if (!mA.hit && !mB.hit) return {};
    if (mA.hit && (!mB.hit || mA.penetration >= mB.penetration)) return mA;
    return mB;
}

CollisionManifold Contain(const WorldOBB& a, const WorldCylinder& c)
{
    // OBB inside cylinder: all corners must satisfy cylinder constraints (radial + caps).
    CollisionManifold m{};

    const glm::vec3 ab = c.b - c.a;
    const float abLenSq = glm::dot(ab, ab);
    if (abLenSq <= kEps) return m;

    const float height = std::sqrt(abLenSq);
    const glm::vec3 axis = ab / height;

    glm::vec3 corners[8]; GetOBBCorners(a, corners);

    float worstPen = 0.0f;
    glm::vec3 bestNormal{ 0.0f };

    for (int i = 0; i < 8; i++)
    {
        const glm::vec3 p = corners[i];

        const float t = glm::dot(p - c.a, axis);
        const float tClamped = Clamp(t, 0.0f, height);
        const glm::vec3 closestAxis = c.a + axis * tClamped;

        const glm::vec3 radial = p - closestAxis;
        const float rLen = std::sqrt(std::max(glm::dot(radial, radial), kEps));

        // violations: outside side wall or outside caps
        const float sideV = rLen - c.radius;          // >0 means outside
        const float capV = std::max(-t, t - height); // >0 means beyond caps

        if (sideV <= 0.0f && capV <= 0.0f) continue; // this corner inside

        if (sideV >= capV)
        {
            const glm::vec3 outward = radial / rLen;
            const float pen = sideV;
            if (pen > worstPen) { worstPen = pen; bestNormal = -outward; }
        }
        else
        {
            // beyond caps: push inward along axis
            const glm::vec3 n = (t < 0.0f) ? axis : -axis;
            const float pen = capV;
            if (pen > worstPen) { worstPen = pen; bestNormal = n; }
        }
    }

    if (worstPen <= 0.0f) return m;

    m.hit = true;
    m.normal = bestNormal;
    m.penetration = worstPen;
    m.contactPoint = a.center;
    return m;
}