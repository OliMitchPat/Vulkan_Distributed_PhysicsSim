#pragma once
#include "Collider.h"

class Sphere : public Collider
{
public:
    Sphere() = default;
    Sphere(const Vector3& centre, double radius);

    double GetRadius() const { return _Radius; }

    bool IsInside(const Vector3& point) const override;
    bool Intersects(const Line& line) const override;
    bool CollideWith(const Sphere& other) const;
    bool IntersectsInfiniteLine(const Vector3& linePoint, const Vector3& lineDir) const;

private:
    double _Radius{ 0.0 };
};
