#pragma once
#include "World.h"
#include "Components.h"

/*class PhysicsSystem
{
public:
    void Update(World& world, float dt)
    {
        const glm::vec3 gravity(0.0f, -9.81f, 0.0f);

        world.forEach<TransformComponent, PhysicsComponent>(
            [&](Entity, TransformComponent& tr, PhysicsComponent& phys)
            {
                // gravity force = m * g
                phys.body.AddForce(gravity * phys.body.Mass());

                phys.body.Integrate(dt, m_integrator);
                tr.position = phys.body.Position();
            });
    }
    void SetIntegrator(IntegratorType type) { m_integrator = type; }
    IntegratorType GetIntegrator() const { return m_integrator; }

private:
    IntegratorType m_integrator = IntegratorType::Euler;
};*/