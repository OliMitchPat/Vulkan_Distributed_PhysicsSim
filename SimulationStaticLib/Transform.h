#pragma once
#include <glm/glm.hpp>

// Physics-friendly transform:
// - position in world space
// - rotation as an orthonormal 3x3 basis (no scale/shear)
// - uniform scale stored separately (we currently require uniform scaling)
struct Transform
{
    glm::vec3 position{ 0.0f };
    glm::mat3 rotation{ 1.0f }; // identity basis
    float scale = 1.0f;

    Transform() = default;

    Transform(const glm::vec3& pos, const glm::mat3& rot, float uniformScale = 1.0f)
        : position(pos), rotation(rot), scale(uniformScale)
    {
    }

    // Local -> world (rotation + uniform scale + translation)
    glm::vec3 TransformPoint(const glm::vec3& localPoint) const
    {
        return position + (rotation * (localPoint * scale));
    }

    // Local direction -> world direction (rotation only; no translation)
    glm::vec3 TransformDirection(const glm::vec3& localDir) const
    {
        return rotation * localDir;
    }
};