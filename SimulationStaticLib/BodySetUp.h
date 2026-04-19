#pragma once

#include "RigidBody.h"
#include "Volume.h"
#include "Inertia.h"

#include <glm/glm.hpp>
#include <algorithm>

inline float MassFromDensity(float density, float volume)
{
    const float d = std::max(0.0f, density);
    const float v = std::max(0.0f, volume);
    return d * v;
}

inline void SetStaticBody(RigidBody& b)
{
    b.SetMotionType(BodyMotionType::Static);
    // SetMotionType(Static) already forces invMass/inertia to 0, but these are harmless:
    b.SetMass(0.0f);
    b.SetInverseInertiaBody(glm::mat3(0.0f));
}

inline void SetKinematicBody(RigidBody& b)
{
    b.SetMotionType(BodyMotionType::Kinematic);
    b.SetMass(0.0f);
    b.SetInverseInertiaBody(glm::mat3(0.0f));
}

// ----- Sphere -----
inline void SetupSphereBody(RigidBody& b, float density, float radius)
{
    if (density <= 0.0f) { SetStaticBody(b); return; }

    b.SetMotionType(BodyMotionType::Dynamic);

    const float mass = MassFromDensity(density, Volume_Sphere(radius));
    b.SetMass(mass);

    const glm::mat3 I = InertiaTensor_Sphere(mass, radius);
    b.SetInverseInertiaBody(SafeInverseInertia(I));
}

// ----- Cuboid -----
inline void SetupCuboidBody(RigidBody& b, float density, const glm::vec3& size)
{
    if (density <= 0.0f) { SetStaticBody(b); return; }

    b.SetMotionType(BodyMotionType::Dynamic);

    const float mass = MassFromDensity(density, Volume_Cuboid(size));
    b.SetMass(mass);

    const glm::mat3 I = InertiaTensor_Cuboid(mass, size);
    b.SetInverseInertiaBody(SafeInverseInertia(I));
}

// ----- Cylinder (local Y axis) -----
inline void SetupCylinderBody(RigidBody& b, float density, float radius, float height)
{
    if (density <= 0.0f) { SetStaticBody(b); return; }

    b.SetMotionType(BodyMotionType::Dynamic);

    const float mass = MassFromDensity(density, Volume_Cylinder(radius, height));
    b.SetMass(mass);

    const glm::mat3 I = InertiaTensor_CylinderY(mass, radius, height);
    b.SetInverseInertiaBody(SafeInverseInertia(I));
}

// ----- Capsule (local Y axis) -----
inline void SetupCapsuleBody(RigidBody& b, float density, float radius, float height)
{
    if (density <= 0.0f) { SetStaticBody(b); return; }

    b.SetMotionType(BodyMotionType::Dynamic);

    const float mass = MassFromDensity(density, Volume_Capsule(radius, height));
    b.SetMass(mass);

    const glm::mat3 I = InertiaTensor_CapsuleY_Approx(mass, radius, height);
    b.SetInverseInertiaBody(SafeInverseInertia(I));
}