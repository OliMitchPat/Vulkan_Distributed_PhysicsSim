#pragma once

#include <glm/glm.hpp>

#include "World.h"
#include "Components.h"
#include "RigidBody.h"
#include "BodySetup.h"
#include "ShapeData.h"

class PhysicsSystem
{
public:
    void Update(World& world, float dt)
    {
        const glm::vec3 gravity(0.0f, -9.81f, 0.0f);

        // --------------------------------------------------
        // 0. INITIALIZATION PASS (runs once per entity)
        // --------------------------------------------------
        world.forEach<PhysicsComponent, TransformComponent>(
            [&](Entity e, PhysicsComponent& phys, TransformComponent& tr)
            {
                if (phys.initialized) return;

                phys.body.SetPosition(tr.position);

                glm::quat q = glm::quat(tr.rotation);
                phys.body.SetOrientation(q);

                // Determine whether this entity is a container (must never move)
                bool isContainer = false;
                if (auto* shape = world.getComponent<ShapeComponent>(e))
                {
                    isContainer = (shape->collisionType == CollisionType::CONTAINER);

                    std::visit([&](auto&& s)
                        {
                            using T = std::decay_t<decltype(s)>;

                            if constexpr (std::is_same_v<T, SphereShape>)
                                SetupSphereBody(phys.body, phys.density, s.radius);
                            else if constexpr (std::is_same_v<T, CuboidShape>)
                                SetupCuboidBody(phys.body, phys.density, s.size);
                            else if constexpr (std::is_same_v<T, CylinderShape>)
                                SetupCylinderBody(phys.body, phys.density, s.radius, s.height);
                            else if constexpr (std::is_same_v<T, CapsuleShape>)
                                SetupCapsuleBody(phys.body, phys.density, s.radius, s.height);
                            else if constexpr (std::is_same_v<T, PlaneShape>)
                                SetStaticBody(phys.body);

                        }, shape->shape);
                }

                if (phys.behaviour == BodyMotionType::Static)
                {
                    phys.body.SetMotionType(BodyMotionType::Static);
                }
                else if (isContainer)
                {
                    phys.body.SetMotionType(BodyMotionType::Static);
                }
                else if (world.getComponent<AnimatedPathComponent>(e))
                {
                    phys.body.SetMotionType(BodyMotionType::Kinematic);
                }
                else if (auto* owner = world.getComponent<OwnerComponent>(e))
                {
                    if (owner->ownerId >= 0)
                    {
                        const bool isOwned = (owner->ownerId == (m_localPeerId - 1));

                        phys.body.SetMotionType(
                            isOwned ? BodyMotionType::Dynamic
                            : BodyMotionType::Kinematic
                        );
                    }
                    else
                    {
                        // ownerId == -1 means this is not an owned simulated object.
                        // For non-animated, non-container objects, treat it as static.
                        phys.body.SetMotionType(BodyMotionType::Static);
                    }
                }
                else
                {
                    // No owner component means this object is not a network-owned simulated body.
                    // Static scene obstacles usually end up here.
                    phys.body.SetMotionType(BodyMotionType::Static);
                }

                // Damping: only meaningful for dynamic bodies
                if (phys.body.IsDynamic())
                {
                    phys.body.SetLinearDamping(0.03f);
                    phys.body.SetAngularDamping(0.08f);
                }
                else
                {
                    phys.body.SetLinearDamping(0.0f);
                    phys.body.SetAngularDamping(0.0f);
                }

                // Optional initial velocity (used by spawned objects)
                if (auto* vel = world.getComponent<VelocityComponent>(e))
                {
                    phys.body.SetLinearVelocity(vel->linearVelocity);
                    phys.body.SetAngularVelocity(vel->angularVelocity);
                }
                printf(
                    "[PhysicsInit] entity=%u density=%.3f mass=%.3f invMass=%.6f restitution=%.3f sf=%.3f df=%.3f\n",
                    (unsigned)e,
                    phys.density,
                    phys.body.Mass(),
                    phys.body.InverseMass(),
                    phys.restitution,
                    phys.staticFriction,
                    phys.dynamicFriction
                );

                phys.initialized = true;
            });

        // --------------------------------------------------
        // 1. APPLY FORCES (owned dynamics only)
        // --------------------------------------------------
        world.forEach<PhysicsComponent>(
            [&](Entity, PhysicsComponent& phys)
            {
                if (!phys.body.IsDynamic()) return;

                if (m_gravityEnabled)
                    phys.body.AddForce(gravity * phys.body.Mass());
            });

        // --------------------------------------------------
        // 2. INTEGRATE (owned dynamics only)
        // --------------------------------------------------
        world.forEach<PhysicsComponent>(
            [&](Entity, PhysicsComponent& phys)
            {
                if (!phys.body.IsDynamic()) return;
                phys.body.Integrate(dt, m_integrator);
            });

        // --------------------------------------------------
        // 3. SYNC BACK TO ECS
        //
        // For Milestone 5:
        // - Owned dynamics should drive transforms.
        // - Replicas (kinematic) will be driven by received snapshots later.
        // --------------------------------------------------
        world.forEach<TransformComponent, PhysicsComponent>(
            [&](Entity e, TransformComponent& tr, PhysicsComponent& phys)
            {
                if (!phys.body.IsDynamic())
                    return;

                tr.position = phys.body.Position();
                tr.rotation = glm::eulerAngles(phys.body.Orientation());

                if (auto* vel = world.getComponent<VelocityComponent>(e))
                {
                    vel->linearVelocity = phys.body.LinearVelocity();
                    vel->angularVelocity = phys.body.AngularVelocity();
                }
            });
    }

    // --------------------------------------------------
    // Config
    // --------------------------------------------------
    void SetIntegrator(IntegratorType type) { m_integrator = type; }
    IntegratorType GetIntegrator() const { return m_integrator; }

    void SetGravityEnabled(bool enabled) { m_gravityEnabled = enabled; }
    bool IsGravityEnabled() const { return m_gravityEnabled; }

    // Milestone 5: used to decide authoritative ownership
    void SetLocalPeerId(int peerId_1_to_4) { m_localPeerId = peerId_1_to_4; }
    int  GetLocalPeerId() const { return m_localPeerId; }

private:
    IntegratorType m_integrator = IntegratorType::Euler;
    bool m_gravityEnabled = true;

    int m_localPeerId = 1; // config peer_id (1..4)
};
