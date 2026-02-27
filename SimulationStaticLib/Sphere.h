#pragma once
#include "Collider.h"

class Sphere : public Collider
{
public:
    Sphere() = default;
    Sphere(const glm::vec3& centre, float radius);

    float GetRadius() const { return m_radius; }

    bool IsInside(const glm::vec3& point) const override;
    bool Intersects(const Line& line) const override;
    bool CollideWith(const Sphere& other) const;
    bool IntersectsInfiniteLine(const glm::vec3& linePoint, const glm::vec3& lineDir) const;

private:
    float m_radius{ 0.0f };
};