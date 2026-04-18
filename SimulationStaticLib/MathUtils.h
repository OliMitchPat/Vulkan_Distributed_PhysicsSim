#pragma once
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

// Keep a small epsilon for robust comparisons
constexpr float kEps = 1e-6f;

// ----- scalar helpers -----

inline float Clamp(float x, float lo, float hi)
{
    return std::max(lo, std::min(x, hi));
}

inline float Saturate(float x) { return Clamp(x, 0.0f, 1.0f); }

inline bool NearlyZero(float x, float eps = kEps) { return std::abs(x) <= eps; }

// ----- vec helpers -----

inline glm::vec3 SafeNormalize(const glm::vec3& v, const glm::vec3& fallback = glm::vec3(1, 0, 0))
{
    const float len2 = glm::dot(v, v);
    if (len2 <= kEps * kEps) return fallback;
    return v * (1.0f / std::sqrt(len2));
}

// Closest point on infinite line: P0 + t*D  (D can be any non-zero vector)
inline glm::vec3 ClosestPointOnLine(
    const glm::vec3& point,
    const glm::vec3& linePoint,
    const glm::vec3& lineDir,
    float* outT = nullptr)
{
    const float lenSq = glm::dot(lineDir, lineDir);
    if (lenSq <= kEps) {
        if (outT) *outT = 0.0f;
        return linePoint;
    }

    const float t = glm::dot(point - linePoint, lineDir) / lenSq;
    if (outT) *outT = t;
    return linePoint + t * lineDir;
}

// Closest point on segment AB. Returns point; optionally returns t in [0,1].
inline glm::vec3 ClosestPointOnSegment(
    const glm::vec3& point,
    const glm::vec3& a,
    const glm::vec3& b,
    float* outT = nullptr)
{
    const glm::vec3 ab = b - a;
    const float abLenSq = glm::dot(ab, ab);

    if (abLenSq <= kEps) {
        if (outT) *outT = 0.0f;
        return a;
    }

    float t = glm::dot(point - a, ab) / abLenSq;
    t = Saturate(t);

    if (outT) *outT = t;
    return a + t * ab;
}

// Signed distance from point to plane (planePoint, unit normal). Positive on normal side.
inline float SignedDistanceToPlane(
    const glm::vec3& point,
    const glm::vec3& planePoint,
    const glm::vec3& planeNormalUnit)
{
    return glm::dot(point - planePoint, planeNormalUnit);
}

// Absolute distance from point to plane (planePoint, unit normal).
inline float DistanceToPlane(
    const glm::vec3& point,
    const glm::vec3& planePoint,
    const glm::vec3& planeNormalUnit)
{
    return std::abs(SignedDistanceToPlane(point, planePoint, planeNormalUnit));
}

// Returns uniform scale factor if scale is ~uniform; otherwise clamps/throws depending on strictness.
// For now: strict uniform scaling as agreed.
inline float RequireUniformScale(const glm::vec3& s, float eps = 1e-3f)
{
    // Treat negative scale as unsupported for physics (mirroring breaks normals).
    // If you ever want to support it, handle sign separately.
    if (s.x <= 0.0f || s.y <= 0.0f || s.z <= 0.0f)
        throw std::invalid_argument("Non-positive scale is not supported for physics colliders");

    const float maxv = std::max(s.x, std::max(s.y, s.z));
    const float minv = std::min(s.x, std::min(s.y, s.z));

    if ((maxv - minv) > eps)
        throw std::invalid_argument("Non-uniform scaling is not supported for physics colliders");

    // Any component is fine since they are ~equal
    return s.x;
}

// Closest points between two segments P1Q1 and P2Q2.
// Returns closest points c1 on segment1 and c2 on segment2.
// Implementation based on standard geometric derivation (robust with degenerates).
inline void ClosestPointsBetweenSegments(
    const glm::vec3& p1, const glm::vec3& q1,
    const glm::vec3& p2, const glm::vec3& q2,
    glm::vec3& c1Out, glm::vec3& c2Out,
    float* sOut = nullptr, float* tOut = nullptr)
{
    const glm::vec3 d1 = q1 - p1; // direction segment1
    const glm::vec3 d2 = q2 - p2; // direction segment2
    const glm::vec3 r = p1 - p2;

    const float a = glm::dot(d1, d1); // squared length seg1
    const float e = glm::dot(d2, d2); // squared length seg2
    const float f = glm::dot(d2, r);

    float s = 0.0f;
    float t = 0.0f;

    // Handle degenerate cases
    if (a <= kEps && e <= kEps)
    {
        // both segments are points
        c1Out = p1;
        c2Out = p2;
        if (sOut) *sOut = 0.0f;
        if (tOut) *tOut = 0.0f;
        return;
    }

    if (a <= kEps)
    {
        // seg1 is point
        s = 0.0f;
        t = Saturate(f / e);
    }
    else
    {
        const float c = glm::dot(d1, r);

        if (e <= kEps)
        {
            // seg2 is point
            t = 0.0f;
            s = Saturate(-c / a);
        }
        else
        {
            const float b = glm::dot(d1, d2);
            const float denom = a * e - b * b;

            if (denom != 0.0f)
                s = Saturate((b * f - c * e) / denom);
            else
                s = 0.0f; // parallel

            t = (b * s + f) / e;

            if (t < 0.0f)
            {
                t = 0.0f;
                s = Saturate(-c / a);
            }
            else if (t > 1.0f)
            {
                t = 1.0f;
                s = Saturate((b - c) / a);
            }
        }
    }

    c1Out = p1 + d1 * s;
    c2Out = p2 + d2 * t;

    if (sOut) *sOut = s;
    if (tOut) *tOut = t;
}

// Closest point on a world-space OBB to a point p.
inline glm::vec3 ClosestPointOnOBB(
    const glm::vec3& p,
    const glm::vec3& center,
    const glm::vec3& axisX,
    const glm::vec3& axisY,
    const glm::vec3& axisZ,
    const glm::vec3& halfExtents)
{
    glm::vec3 d = p - center;
    glm::vec3 q = center;

    float x = glm::dot(d, axisX);
    x = Clamp(x, -halfExtents.x, halfExtents.x);
    q += axisX * x;

    float y = glm::dot(d, axisY);
    y = Clamp(y, -halfExtents.y, halfExtents.y);
    q += axisY * y;

    float z = glm::dot(d, axisZ);
    z = Clamp(z, -halfExtents.z, halfExtents.z);
    q += axisZ * z;

    return q;
}