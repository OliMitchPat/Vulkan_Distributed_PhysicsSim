#pragma once

#include "RigidBody.h"
#include "BodyDispatcher.h"
#include "FlatbufferConvert.h"

#include <algorithm>
#include <string>
#include <type_traits>
#include <variant>

// Scale the shape dimensions by uniform scale derived from the transform.
// Must match the policy used in Scenario_FlatbufferScene (max abs component).
inline ShapeData ScaleShapeUniform(const ShapeData& shape, float u)
{
    return std::visit([u](auto&& s) -> ShapeData
        {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, SphereShape>)
                return SphereShape{ s.radius * u };
            if constexpr (std::is_same_v<T, CuboidShape>)
                return CuboidShape{ s.size * u };
            if constexpr (std::is_same_v<T, CylinderShape>)
                return CylinderShape{ s.radius * u, s.height * u };
            if constexpr (std::is_same_v<T, CapsuleShape>)
                return CapsuleShape{ s.radius * u, s.height * u };
            if constexpr (std::is_same_v<T, PlaneShape>)
                return s; // plane: scale doesn't change the normal
            return s;
        }, shape);
}

template <typename DensityLookupFn>
inline void SetupRigidBodyFromFlatbufferObject(
    RigidBody& body,
    const Simulation::Object* obj,
    DensityLookupFn&& getDensityByMaterialName)
{
    if (!obj) return;

    // Pose
    const auto* t = obj->transform();
    body.SetPosition(SimIO::ToPosition(t));
    body.SetOrientation(SimIO::ToOrientationQuat(t));

    const BehaviourType behaviour = SimIO::ToBehaviourType(obj->behaviour_type());
    const CollisionType collisionType = SimIO::ToCollisionType(obj->collision_type());

    // Material density (default 0 => static by your BodySetup policy)
    float density = 0.0f;
    if (obj->material())
        density = std::max(0.0f, getDensityByMaterialName(obj->material()->str()));

    // Shape (with defaults) + IMPORTANT: apply uniform scale
    const ShapeData rawShape = SimIO::ToShapeData(obj);
    const float uniformScale = SimIO::ToUniformScale_MaxAbs(t, 1.0f);
    const ShapeData shape = ScaleShapeUniform(rawShape, uniformScale);

    // Initial velocities for simulated objects
    if (behaviour == BehaviourType::Simulated)
    {
        const auto* sim = obj->behaviour_as_SimulatedObject();
        const auto* state = (sim) ? sim->initial_state() : nullptr;

        body.SetLinearVelocity(SimIO::ToLinearVelocity(state));
        body.SetAngularVelocity(SimIO::ToAngularVelocityRad(state));
    }
    else
    {
        body.SetLinearVelocity(glm::vec3(0.0f));
        body.SetAngularVelocity(glm::vec3(0.0f));
    }

    SetupBodyFromShape(body, shape, density, behaviour, collisionType);
}