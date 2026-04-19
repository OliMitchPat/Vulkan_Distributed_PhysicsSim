#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cmath>
#include <string>

#include "ShapeData.h"

// TODO: update include path once you generate it with flatc
#include "Scene_generated.h"

namespace SimIO
{
    // Defaults (tune as you like)
    inline constexpr float kDefaultRadius = 0.5f;
    inline constexpr float kDefaultHeight = 1.0f;
    inline constexpr glm::vec3 kDefaultScale{ 1.0f, 1.0f, 1.0f };
    inline constexpr glm::vec3 kDefaultCuboidSize{ 1.0f, 1.0f, 1.0f };
    inline constexpr glm::vec3 kDefaultPlaneNormal{ 0.0f, 1.0f, 0.0f };

    inline float DegToRad(float deg) { return deg * (3.14159265358979323846f / 180.0f); }

    inline float ClampPositiveOrDefault(float v, float def) { return (v > 0.0f) ? v : def; }

    inline glm::vec3 SafeNormalizeOrDefault(const glm::vec3& v, const glm::vec3& def)
    {
        const float len2 = glm::dot(v, v);
        if (len2 < 1e-10f) return def;
        return v * (1.0f / std::sqrt(len2));
    }

    // --- Euler (deg) -> quaternion ---
    inline glm::quat QuatFromEulerDeg_YawPitchRoll(float yawDeg, float pitchDeg, float rollDeg)
    {
        const float yaw = DegToRad(yawDeg);
        const float pitch = DegToRad(pitchDeg);
        const float roll = DegToRad(rollDeg);

        const glm::quat qYaw = glm::angleAxis(yaw, glm::vec3(0, 1, 0));
        const glm::quat qPitch = glm::angleAxis(pitch, glm::vec3(1, 0, 0));
        const glm::quat qRoll = glm::angleAxis(roll, glm::vec3(0, 0, 1));

        return glm::normalize(qYaw * qPitch * qRoll);
    }

    // These will likely need small edits after flatc depending on accessors:

    inline BehaviourType ToBehaviourType(const Simulation::Behaviour t)
    {
        switch (t)
        {
        case Simulation::Behaviour_StaticObject:    return BehaviourType::Static;
        case Simulation::Behaviour_AnimatedObject:  return BehaviourType::Animated;
        case Simulation::Behaviour_SimulatedObject: return BehaviourType::Simulated;
        default:                                    return BehaviourType::Static;
        }
    }

    inline CollisionType ToCollisionType(Simulation::CollisionType t)
    {
        return (t == Simulation::CollisionType_CONTAINER) ? CollisionType::CONTAINER : CollisionType::SOLID;
    }

    // Object->shape -> ShapeData
    inline ShapeData ToShapeData(const Simulation::Object* obj)
    {
        if (!obj || !obj->shape())
            return SphereShape{ kDefaultRadius };

        switch (obj->shape_type())
        {
        case Simulation::Shape_Sphere:
        {
            auto s = obj->shape_as_Sphere();
            const float r = s ? ClampPositiveOrDefault(s->radius(), kDefaultRadius) : kDefaultRadius;
            return SphereShape{ r };
        }
        case Simulation::Shape_Cuboid:
        {
            auto c = obj->shape_as_Cuboid();
            glm::vec3 size = kDefaultCuboidSize;
            if (c && c->size())
                size = glm::vec3(c->size()->x(), c->size()->y(), c->size()->z());

            size.x = ClampPositiveOrDefault(size.x, kDefaultCuboidSize.x);
            size.y = ClampPositiveOrDefault(size.y, kDefaultCuboidSize.y);
            size.z = ClampPositiveOrDefault(size.z, kDefaultCuboidSize.z);

            return CuboidShape{ size };
        }
        case Simulation::Shape_Cylinder:
        {
            auto cy = obj->shape_as_Cylinder();
            const float r = cy ? ClampPositiveOrDefault(cy->radius(), kDefaultRadius) : kDefaultRadius;
            const float h = cy ? ClampPositiveOrDefault(cy->height(), kDefaultHeight) : kDefaultHeight;
            return CylinderShape{ r, h };
        }
        case Simulation::Shape_Capsule:
        {
            auto ca = obj->shape_as_Capsule();
            const float r = ca ? ClampPositiveOrDefault(ca->radius(), kDefaultRadius) : kDefaultRadius;
            const float h = ca ? ClampPositiveOrDefault(ca->height(), kDefaultHeight) : kDefaultHeight;
            return CapsuleShape{ r, h };
        }
        case Simulation::Shape_Plane:
        {
            auto p = obj->shape_as_Plane();
            glm::vec3 n = kDefaultPlaneNormal;
            if (p && p->normal())
                n = glm::vec3(p->normal()->x(), p->normal()->y(), p->normal()->z());

            n = SafeNormalizeOrDefault(n, kDefaultPlaneNormal);
            return PlaneShape{ n };
        }
        default:
            return SphereShape{ kDefaultRadius };
        }
    }

    inline glm::vec3 ToPosition(const Simulation::Transform* t)
    {
        if (!t || !t->position()) return glm::vec3(0.0f);
        return glm::vec3(t->position()->x(), t->position()->y(), t->position()->z());
    }

    inline glm::quat ToOrientationQuat(const Simulation::Transform* t)
    {
        if (!t || !t->orientation()) return glm::quat(1, 0, 0, 0);
        const auto* e = t->orientation();
        return QuatFromEulerDeg_YawPitchRoll(e->yaw(), e->pitch(), e->roll());
    }

    inline glm::vec3 ToLinearVelocity(const Simulation::PhysicsState* s)
    {
        if (!s || !s->linear_velocity()) return glm::vec3(0.0f);
        return glm::vec3(s->linear_velocity()->x(), s->linear_velocity()->y(), s->linear_velocity()->z());
    }

    inline glm::vec3 ToAngularVelocityRad(const Simulation::PhysicsState* s)
    {
        if (!s || !s->angular_velocity()) return glm::vec3(0.0f);
        const float x = DegToRad(s->angular_velocity()->x());
        const float y = DegToRad(s->angular_velocity()->y());
        const float z = DegToRad(s->angular_velocity()->z());
        return glm::vec3(x, y, z);
    }
}