#pragma once
#include <glm/glm.hpp>
#include "Line.h"

class Collider
{
public:
    explicit Collider(const glm::vec3& position = {}) : m_position(position) {}
    virtual ~Collider() = default;

    const glm::vec3& GetPosition() const { return m_position; }
    void SetPosition(const glm::vec3& p) { m_position = p; }

    virtual bool IsInside(const glm::vec3& point) const = 0;
    virtual bool Intersects(const Line& line) const = 0;

protected:
    glm::vec3 m_position{};
};