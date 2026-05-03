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

                // ---- Sync Transform -> RigidBody ----
                phys.body.SetPosition(tr.position);

                // Convert Euler (stored in Transform) -> quaternion
                glm::quat q = glm::quat(tr.rotation);
                phys.body.SetOrientation(q);

                // ---- Setup body from ShapeComponent if available ----
                if (auto* shape = world.getComponent<ShapeComponent>(e))
                {
                    std::visit([&](auto&& s)
                        {
                            using T = std::decay_t<decltype(s)>;

                            if constexpr (std::is_same_v<T, SphereShape>)
                            {
                                SetupSphereBody(phys.body, phys.density, s.radius);
                            }
                            else if constexpr (std::is_same_v<T, CuboidShape>)
                            {
                                SetupCuboidBody(phys.body, phys.density, s.size);
                            }
                            else if constexpr (std::is_same_v<T, CylinderShape>)
                            {
                                SetupCylinderBody(phys.body, phys.density, s.radius, s.height);
                            }
                            else if constexpr (std::is_same_v<T, CapsuleShape>)
                            {
                                SetupCapsuleBody(phys.body, phys.density, s.radius, s.height);
                            }
                            else if constexpr (std::is_same_v<T, PlaneShape>)
                            {
                                SetStaticBody(phys.body);
                            }

                        }, shape->shape);
                }
                phys.initialized = true;
            });

        // --------------------------------------------------
        // 1. APPLY FORCES
        // --------------------------------------------------
        world.forEach<PhysicsComponent>(
            [&](Entity, PhysicsComponent& phys)
            {
                if (!phys.body.IsDynamic()) return;

                if (m_gravityEnabled)
                    phys.body.AddForce(gravity * phys.body.Mass());
            });

        // --------------------------------------------------
        // 2. INTEGRATE
        // --------------------------------------------------
        world.forEach<PhysicsComponent>(
            [&](Entity, PhysicsComponent& phys)
            {
                phys.body.Integrate(dt, m_integrator);
            });

        // --------------------------------------------------
        // 3. SYNC BACK TO ECS
        // --------------------------------------------------
        world.forEach<TransformComponent, PhysicsComponent>(
            [&](Entity e, TransformComponent& tr, PhysicsComponent& phys)
            {
                // Update transform from physics
                tr.position = phys.body.Position();
                tr.rotation = glm::eulerAngles(phys.body.Orientation());

                // Optional: write back velocities
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

private:
    IntegratorType m_integrator = IntegratorType::Euler;
    bool m_gravityEnabled = true;
};