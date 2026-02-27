#pragma once
#include "Collider.h"
class Sphere;

class Plane : public Collider
{
public:
    Plane() = default;
    Plane(const glm::vec3& pointOnPlane, const glm::vec3& normal);

    const glm::vec3& GetNormal() const { return m_normal; }

    bool IsInside(const glm::vec3& point) const override;
    bool Intersects(const Line& line) const override;
    bool Intersects(const Sphere& sphere) const;

private:
    glm::vec3 m_normal{ 0, 1, 0 };
};