#pragma once
#include <glm/glm.hpp>
#include <algorithm>

// Returns body-space inertia tensor (NOT inverse) about center of mass,
// aligned to the shape's local axes.
inline glm::mat3 InertiaTensor_Sphere(float mass, float radius)
{
    // I = 2/5 m r^2
    const float i = (2.0f / 5.0f) * mass * radius * radius;
    return glm::mat3(i, 0, 0, 0, i, 0, 0, 0, i);
}

inline glm::mat3 InertiaTensor_Cuboid(float mass, const glm::vec3& size)
{
    // size = full widths (x,y,z). For box: Ixx = 1/12 m (y^2 + z^2), etc.
    const float x2 = size.x * size.x;
    const float y2 = size.y * size.y;
    const float z2 = size.z * size.z;

    const float Ixx = (1.0f / 12.0f) * mass * (y2 + z2);
    const float Iyy = (1.0f / 12.0f) * mass * (x2 + z2);
    const float Izz = (1.0f / 12.0f) * mass * (x2 + y2);

    return glm::mat3(Ixx, 0, 0, 0, Iyy, 0, 0, 0, Izz);
}

inline glm::mat3 InertiaTensor_CylinderY(float mass, float radius, float height)
{
    // Solid cylinder aligned to Y axis:
    // Iyy = 1/2 m r^2
    // Ixx = Izz = 1/12 m (3r^2 + h^2)
    const float r2 = radius * radius;
    const float h2 = height * height;

    const float Iyy = 0.5f * mass * r2;
    const float Ixx = (1.0f / 12.0f) * mass * (3.0f * r2 + h2);
    const float Izz = Ixx;

    return glm::mat3(Ixx, 0, 0, 0, Iyy, 0, 0, 0, Izz);
}

// Capsule inertia is more complex to do exactly.
// For coursework, a good-quality approximation is cylinder + two hemispheres
// using the parallel axis theorem. Here is a safe approximation:
// treat capsule as a cylinder (height) plus a sphere (radius) using total height.
inline glm::mat3 InertiaTensor_CapsuleY_Approx(float mass, float radius, float height)
{
    // height here should be the "cylinder section" height (as per your spec).
    // We'll approximate distribution: 2/3 mass in cylinder, 1/3 in sphere.
    const float mCyl = mass * (2.0f / 3.0f);
    const float mSph = mass * (1.0f / 3.0f);

    const glm::mat3 Ic = InertiaTensor_CylinderY(mCyl, radius, height);
    const glm::mat3 Is = InertiaTensor_Sphere(mSph, radius);

    return Ic + Is;
}

inline glm::mat3 SafeInverseInertia(const glm::mat3& I)
{
    // If any diagonal is ~0, treat as non-rotating (infinite inertia).
    // Since our inertia tensors are diagonal, invert diagonals safely.
    glm::mat3 inv(0.0f);

    const float eps = 1e-8f;
    inv[0][0] = (std::abs(I[0][0]) > eps) ? (1.0f / I[0][0]) : 0.0f;
    inv[1][1] = (std::abs(I[1][1]) > eps) ? (1.0f / I[1][1]) : 0.0f;
    inv[2][2] = (std::abs(I[2][2]) > eps) ? (1.0f / I[2][2]) : 0.0f;
    return inv;
}
