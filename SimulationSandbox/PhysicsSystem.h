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

                phys.body.AddForce(gravity * phys.body.Mass());
            });

        // 2. Integrate AFTER collision resolution
        world.forEach<TransformComponent, PhysicsComponent>(
            [&](Entity, TransformComponent& tr, PhysicsComponent& phys)
            {
                phys.body.Integrate(dt, m_integrator);
                tr.position = phys.body.Position();
            });
    }

    void SetIntegrator(IntegratorType type) { m_integrator = type; }
    IntegratorType GetIntegrator() const { return m_integrator; }

private:
    IntegratorType m_integrator = IntegratorType::Euler;
};