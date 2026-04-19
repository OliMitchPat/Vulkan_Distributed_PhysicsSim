#pragma once
#include "RigidBody.h"
#include "BodySetup.h"
#include "ShapeData.h"

#include <type_traits>
#include <variant>

inline void SetupBodyFromShape(
    RigidBody& body,
    const ShapeData& shape,
    float density,
    BehaviourType behaviour,
    CollisionType collisionType)
{
    // Behaviour policy
    if (behaviour == BehaviourType::Static) { SetStaticBody(body); return; }
    if (behaviour == BehaviourType::Animated) { SetKinematicBody(body); return; }

    // Simulated containers should act as static colliders (good default)
    if (collisionType == CollisionType::CONTAINER) { SetStaticBody(body); return; }

    // Simulated SOLID
    std::visit([&](auto&& s)
        {
            using T = std::decay_t<decltype(s)>;

            if constexpr (std::is_same_v<T, SphereShape>)
                SetupSphereBody(body, density, s.radius);
            else if constexpr (std::is_same_v<T, CuboidShape>)
                SetupCuboidBody(body, density, s.size);
            else if constexpr (std::is_same_v<T, CylinderShape>)
                SetupCylinderBody(body, density, s.radius, s.height);
            else if constexpr (std::is_same_v<T, CapsuleShape>)
                SetupCapsuleBody(body, density, s.radius, s.height);
            else if constexpr (std::is_same_v<T, PlaneShape>)
            {
                // Infinite plane => static collider
                SetStaticBody(body);
            }
        }, shape);
}