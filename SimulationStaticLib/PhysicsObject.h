#pragma once
#include <glm/glm.hpp>

class PhysicsObject
{
public:
    PhysicsObject() = default;
    explicit PhysicsObject(const glm::mat4& transform) : m_transform(transform) {}

    const glm::mat4& Transform() const { return m_transform; }
    glm::mat4& Transform() { return m_transform; }

    // convenience helpers (optional)
    glm::vec3 Position() const { return glm::vec3(m_transform[3]); }
    void SetPosition(const glm::vec3& p) { m_transform[3] = glm::vec4(p, 1.0f); }

private:
    glm::mat4 m_transform{ 1.0f };
};