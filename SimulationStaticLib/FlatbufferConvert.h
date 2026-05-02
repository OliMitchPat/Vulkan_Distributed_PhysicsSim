#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <string>

#include "ShapeData.h"

#include "Scene_generated.h"
#include "Transform.h"

namespace SimIO
{
    // -----------------------
    // Defaults / sanitizers
    // -----------------------
    inline constexpr float kDefaultRadius = 0.5f;
    inline constexpr float kDefaultHeight = 1.0f;

    inline constexpr glm::vec3 kDefaultCuboidSize{ 1.0f, 1.0f, 1.0f };
    inline constexpr glm::vec3 kDefaultPlaneNormal{ 0.0f, 1.0f, 0.0f };

    inline float DegToRad(float deg)
    {
        return deg * (3.14159265358979323846f / 180.0f);
    }

    inline float ClampPositiveOrDefault(float v, float def)
    {
        return (v > 0.0f) ? v : def;
    }

    inline glm::vec3 SafeNormalizeOrDefault(const glm::vec3& v, const glm::vec3& def)
    {
        const float len2 = glm::dot(v, v);
        if (len2 < 1e-10f) return def;
        return v * (1.0f / std::sqrt(len2));
    }

    // -----------------------
    // Basic struct conversions
    // -----------------------
    inline glm::vec3 ToGlmVec3(const Simulation::Vec3& v)
    {
        return glm::vec3(v.x(), v.y(), v.z());
    }

    inline glm::quat QuatFromEulerDeg_YawPitchRoll(float yawDeg, float pitchDeg, float rollDeg)
    {
        // Spec:
        // yaw   = rotation around Y (degrees)
        // pitch = rotation around X (degrees)
        // roll  = rotation around Z (degrees)
        const float yaw = DegToRad(yawDeg);
        const float pitch = DegToRad(pitchDeg);
        const float roll = DegToRad(rollDeg);

        // Compose yaw(Y) -> pitch(X) -> roll(Z)
        const glm::quat qYaw = glm::angleAxis(yaw, glm::vec3(0, 1, 0));
        const glm::quat qPitch = glm::angleAxis(pitch, glm::vec3(1, 0, 0));
        const glm::quat qRoll = glm::angleAxis(roll, glm::vec3(0, 0, 1));

        return glm::normalize(qYaw * qPitch * qRoll);
    }

    inline glm::quat ToOrientationQuat(const Simulation::RotationEuler& e)
    {
        return QuatFromEulerDeg_YawPitchRoll(e.yaw(), e.pitch(), e.roll());
    }

    // -----------------------
    // Enum/union mappings
    // -----------------------
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
        return (t == Simulation::CollisionType_CONTAINER)
            ? CollisionType::CONTAINER
            : CollisionType::SOLID;
    }

    // -----------------------
    // Object->shape -> ShapeData
    // -----------------------
    inline ShapeData ToShapeData(const Simulation::Object* obj)
    {
        // Correct union "missing" test for your generated file:
        if (!obj || obj->shape_type() == Simulation::Shape_NONE)
            return SphereShape{ kDefaultRadius };

        switch (obj->shape_type())
        {
        case Simulation::Shape_Sphere:
        {
            const auto* s = obj->shape_as_Sphere();
            const float r = (s) ? ClampPositiveOrDefault(s->radius(), kDefaultRadius) : kDefaultRadius;
            return SphereShape{ r };
        }

        case Simulation::Shape_Cuboid:
        {
            const auto* c = obj->shape_as_Cuboid();
            glm::vec3 size = kDefaultCuboidSize;

            // In your generated header: Cuboid::size() returns const Simulation::Vec3* (nullable)
            if (c)
            {
                const Simulation::Vec3* s = c->size();
                if (s) size = ToGlmVec3(*s);
            }

            // sanitize components
            size.x = ClampPositiveOrDefault(size.x, kDefaultCuboidSize.x);
            size.y = ClampPositiveOrDefault(size.y, kDefaultCuboidSize.y);
            size.z = ClampPositiveOrDefault(size.z, kDefaultCuboidSize.z);

            return CuboidShape{ size };
        }

        case Simulation::Shape_Cylinder:
        {
            const auto* cy = obj->shape_as_Cylinder();
            const float r = (cy) ? ClampPositiveOrDefault(cy->radius(), kDefaultRadius) : kDefaultRadius;
            const float h = (cy) ? ClampPositiveOrDefault(cy->height(), kDefaultHeight) : kDefaultHeight;
            return CylinderShape{ r, h };
        }

        case Simulation::Shape_Capsule:
        {
            const auto* ca = obj->shape_as_Capsule();
            const float r = (ca) ? ClampPositiveOrDefault(ca->radius(), kDefaultRadius) : kDefaultRadius;
            const float h = (ca) ? ClampPositiveOrDefault(ca->height(), kDefaultHeight) : kDefaultHeight;
            return CapsuleShape{ r, h };
        }

        case Simulation::Shape_Plane:
        {
            const auto* p = obj->shape_as_Plane();
            glm::vec3 n = kDefaultPlaneNormal;

            // In your generated header: Plane::normal() returns const Simulation::Vec3* (nullable)
            if (p)
            {
                const Simulation::Vec3* nn = p->normal();
                if (nn) n = ToGlmVec3(*nn);
            }

            n = SafeNormalizeOrDefault(n, kDefaultPlaneNormal);
            return PlaneShape{ n };
        }

        default:
            return SphereShape{ kDefaultRadius };
        }
    }

    // -----------------------
    // Transform conversions
    // -----------------------
    inline glm::vec3 ToPosition(const Simulation::Transform* t)
    {
        // In your generated header: Transform is a struct; Object::transform() returns const Transform* (nullable)
        if (!t) return glm::vec3(0.0f);
        return ToGlmVec3(t->position()); // position() returns const Vec3&
    }

    inline glm::quat ToOrientationQuat(const Simulation::Transform* t)
    {
        if (!t) return glm::quat(1, 0, 0, 0);
        return ToOrientationQuat(t->orientation()); // orientation() returns const RotationEuler&
    }

    inline glm::vec3 ToScaleOrDefault(const Simulation::Transform* t, const glm::vec3& def = glm::vec3(1.0f))
    {
        if (!t) return def;

        glm::vec3 s = ToGlmVec3(t->scale()); // scale() returns const Vec3&
        // Avoid degenerate zero scale
        s.x = (std::abs(s.x) > 1e-8f) ? s.x : def.x;
        s.y = (std::abs(s.y) > 1e-8f) ? s.y : def.y;
        s.z = (std::abs(s.z) > 1e-8f) ? s.z : def.z;
        return s;
    }

    inline float ToUniformScale_MaxAbs(const Simulation::Transform* t, float def = 1.0f)
    {
        const glm::vec3 s = ToScaleOrDefault(t, glm::vec3(def));

        const float ax = std::abs(s.x);
        const float ay = std::abs(s.y);
        const float az = std::abs(s.z);

        const float u = std::max(ax, std::max(ay, az));
        return (u > 1e-8f) ? u : def;
    }

    inline Transform ToPhysicsTransform(const Simulation::Transform* t)
    {
        Transform out{};
        out.position = ToPosition(t);

        // Use the same quaternion conversion you already have
        const glm::quat q = ToOrientationQuat(t);
        out.rotation = glm::mat3_cast(q);

        out.scale = ToUniformScale_MaxAbs(t, 1.0f);
        return out;
    }

    // -----------------------
    // PhysicsState conversions
    // -----------------------
    inline glm::vec3 ToLinearVelocity(const Simulation::PhysicsState* s)
    {
        // In your generated header: SimulatedObject::initial_state() returns const PhysicsState* (nullable)
        if (!s) return glm::vec3(0.0f);
        return ToGlmVec3(s->linear_velocity()); // returns const Vec3&
    }

    inline glm::vec3 ToAngularVelocityRad(const Simulation::PhysicsState* s)
    {
        if (!s) return glm::vec3(0.0f);

        const glm::vec3 wDeg = ToGlmVec3(s->angular_velocity()); // deg/sec
        return glm::vec3(
            DegToRad(wDeg.x),
            DegToRad(wDeg.y),
            DegToRad(wDeg.z));
    }

    // -----------------------
    // Convenience string helpers
    // -----------------------
    inline std::string ToStdStringOrEmpty(const flatbuffers::String* s)
    {
        return s ? s->str() : std::string{};
    }
} // namespace SimIO