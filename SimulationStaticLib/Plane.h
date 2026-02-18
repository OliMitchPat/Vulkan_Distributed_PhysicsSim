#pragma once
#include "Collider.h"
class Sphere;

class Plane : public Collider
{
public:
    Plane() = default;
    Plane(const Vector3& pointOnPlane, const Vector3& normal);

    const Vector3& GetNormal() const { return _Normal; }

    bool IsInside(const Vector3& point) const override;
    bool Intersects(const Line& line) const override;
    bool Intersects(const Sphere& sphere) const;

private:
    Vector3 _Normal{ 0, 1, 0 }; // should be normalized ideally
};
