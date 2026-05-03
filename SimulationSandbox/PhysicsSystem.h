#include "World.h"
#include "Components.h"
#include "RigidBody.h"

#include <glm/gtc/quaternion.hpp>

class PhysicsSystem
{
public:
    void Update(World& world, float dt)
    {
        const glm::vec3 gravity(0.0f, -9.81f, 0.0f);

        // 1. Apply forces
        world.forEach<PhysicsComponent>([&](Entity, PhysicsComponent& phys)
            {
                if (!phys.body.IsDynamic()) return;

                if (m_gravityEnabled)
                    phys.body.AddForce(gravity * phys.body.Mass());
            });

        // 2. Integrate and sync pose back to TransformComponent
        world.forEach<TransformComponent, PhysicsComponent>(
            [&](Entity, TransformComponent& tr, PhysicsComponent& phys)
            {
                phys.body.Integrate(dt, m_integrator);
                tr.position = phys.body.Position();
                // Sync orientation so the renderer sees up-to-date rotation
                tr.rotation = glm::eulerAngles(phys.body.Orientation());
            });
    }

    void SetIntegrator(IntegratorType type) { m_integrator = type; }
    IntegratorType GetIntegrator() const { return m_integrator; }

    void SetGravityEnabled(bool enabled) { m_gravityEnabled = enabled; }
    bool IsGravityEnabled() const { return m_gravityEnabled; }

private:
    IntegratorType m_integrator = IntegratorType::Euler;
    bool           m_gravityEnabled = true;
};