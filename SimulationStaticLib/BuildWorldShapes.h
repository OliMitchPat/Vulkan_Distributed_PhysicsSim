#pragma once
#include "WorldShapes.h"

#include "Transform.h"
#include "MathUtils.h"

#include "ShapeData.h"

#include <glm/glm.hpp>

// Builds a world-space sphere from a local sphere shape + transform.
inline WorldSphere BuildWorldSphere(const Transform& tr, const SphereShape& s)
{
    WorldSphere out{};
    out.center = tr.position;

    // Enforce uniform scaling (currently required by design)
    out.radius = s.radius * tr.scale;
    return out;
}

inline WorldPlane BuildWorldPlane(const Transform& tr, const PlaneShape& p)
{
    WorldPlane out{};
    out.point = tr.position;

    // Rotate local normal into world, ensure unit length
    out.normal = SafeNormalize(tr.TransformDirection(p.normal), glm::vec3(0, 1, 0));
    return out;
}

inline WorldCapsule BuildWorldCapsule(const Transform& tr, const CapsuleShape& c)
{
    WorldCapsule out{};
    out.radius = c.radius * tr.scale;

    // Capsule aligned to local +Y, rotated by transform
    const glm::vec3 axis = SafeNormalize(tr.TransformDirection(glm::vec3(0, 1, 0)), glm::vec3(0, 1, 0));
    const float halfH = 0.5f * (c.height * tr.scale);

    out.a = tr.position - axis * halfH;
    out.b = tr.position + axis * halfH;
    return out;
}

inline WorldCylinder BuildWorldCylinder(const Transform& tr, const CylinderShape& c)
{
    WorldCylinder out{};
    out.radius = c.radius * tr.scale;

    const glm::vec3 axis = SafeNormalize(tr.TransformDirection(glm::vec3(0, 1, 0)), glm::vec3(0, 1, 0));
    const float halfH = 0.5f * (c.height * tr.scale);

    out.a = tr.position - axis * halfH;
    out.b = tr.position + axis * halfH;
    return out;
}

inline WorldOBB BuildWorldOBB(const Transform& tr, const CuboidShape& b)
{
    WorldOBB out{};
    out.center = tr.position;

    // For an orthonormal basis, using the rotation matrix columns is standard in GLM.
    // GLM mat3 is column-major: m[0], m[1], m[2] are the basis columns.
    out.axisX = SafeNormalize(tr.rotation[0], glm::vec3(1, 0, 0));
    out.axisY = SafeNormalize(tr.rotation[1], glm::vec3(0, 1, 0));
    out.axisZ = SafeNormalize(tr.rotation[2], glm::vec3(0, 0, 1));

    out.halfExtents = 0.5f * (b.size * tr.scale);
    return out;
}