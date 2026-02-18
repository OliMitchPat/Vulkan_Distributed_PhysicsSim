#include "pch.h"
#include "Plane.h"
#include <stdexcept>
#include "MathUtils.h"

Plane::Plane(const Vector3& pointOnPlane, const Vector3& normal)
    : Collider(pointOnPlane), _Normal(normal)
{
    if (_Normal.LengthSquared() == 0.0)
        throw std::invalid_argument("Plane normal must be non-zero");
}

// Convention:
// "Inside" means on the plane or on the side the normal points to.
// i.e. dot(point - planePoint, normal) >= 0
bool Plane::IsInside(const Vector3& point) const
{
    const Vector3 v = point - _Position;
    return v.Dot(_Normal) >= 0.0;
}

bool Plane::Intersects(const Line& line) const
{
    // Segment-plane intersection:
    // Let signed distances be d0 = dot(S - P0, N), d1 = dot(E - P0, N).
    // If they have opposite signs (or either is zero), segment intersects plane.
    const double d0 = (line.Start - _Position).Dot(_Normal);
    const double d1 = (line.End - _Position).Dot(_Normal);

    if (d0 == 0.0 && d1 == 0.0)
        return true; // segment lies in plane

    return (d0 == 0.0) || (d1 == 0.0) || (d0 < 0.0 && d1 > 0.0) || (d0 > 0.0 && d1 < 0.0);
}

bool Plane::Intersects(const Sphere& sphere) const
{
    double d = ClosestDistancePointToPlane(sphere.GetPosition(), _Position, _Normal);
    return d <= sphere.GetRadius();
}
