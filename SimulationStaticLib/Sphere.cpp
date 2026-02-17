#include "pch.h"
#include "Sphere.h"
#include <stdexcept>
#include "MathUtils.h"

Sphere::Sphere(const Vector3& centre, double radius)
    : Collider(centre), _Radius(radius)
{
    if (radius < 0.0)
        throw std::invalid_argument("Sphere radius must be non-negative");
}

bool Sphere::IsInside(const Vector3& point) const
{
    const Vector3 diff = point - _Position;
    return diff.LengthSquared() <= _Radius * _Radius;
}

bool Sphere::Intersects(const Line& line) const
{
    // Segment-sphere intersection:
    // Solve |(S + t*(E-S)) - C|^2 = r^2 for t in [0,1]
    const Vector3 d = line.End - line.Start;         // segment direction
    const Vector3 m = line.Start - _Position;        // from centre to start

    const double a = d.Dot(d);
    const double b = 2.0 * m.Dot(d);
    const double c = m.Dot(m) - _Radius * _Radius;

    // Degenerate segment (Start == End): treat as point test
    if (a == 0.0)
        return m.Dot(m) <= _Radius * _Radius;

    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0)
        return false;

    const double sqrtDisc = std::sqrt(discriminant);
    const double inv2a = 1.0 / (2.0 * a);

    const double t0 = (-b - sqrtDisc) * inv2a;
    const double t1 = (-b + sqrtDisc) * inv2a;

    // intersects segment if either root is within [0,1]
    return (t0 >= 0.0 && t0 <= 1.0) || (t1 >= 0.0 && t1 <= 1.0);
}

bool Sphere::CollideWith(const Sphere& other) const
{
    const Vector3 delta = other.GetPosition() - _Position;
    const double rSum = _Radius + other._Radius;

    return delta.LengthSquared() <= rSum * rSum;
}

bool Sphere::IntersectsInfiniteLine(const Vector3& linePoint, const Vector3& lineDir) const
{
    return ClosestDistancePointToLine(this->GetPosition(), linePoint, lineDir) <= this->GetRadius();
}

