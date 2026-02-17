#pragma once
#include <cmath>

struct Vector3
{
    double x{};
    double y{};
    double z{};

    constexpr Vector3() = default;
    constexpr Vector3(double x, double y, double z) : x(x), y(y), z(z) {}

    // Arithmetic
    constexpr Vector3 operator+(const Vector3& rhs) const { return { x + rhs.x, y + rhs.y, z + rhs.z }; }
    constexpr Vector3 operator-(const Vector3& rhs) const { return { x - rhs.x, y - rhs.y, z - rhs.z }; }
    constexpr Vector3 operator*(double s) const { return { x * s, y * s, z * s }; }
    constexpr Vector3 operator/(double s) const { return { x / s, y / s, z / s }; }

    // Length
    double LengthSquared() const { return x * x + y * y + z * z; }
    double Length() const { return std::sqrt(LengthSquared()); }

    // Dot
    constexpr double Dot(const Vector3& rhs) const { return x * rhs.x + y * rhs.y + z * rhs.z; }
};
