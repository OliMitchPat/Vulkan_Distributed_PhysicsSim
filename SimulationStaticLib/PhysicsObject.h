#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

enum class IntegratorType
{
    Euler,
    SemiImplicitEuler,
    // RK4, // later
};

class PhysicsObject
{
public:
    PhysicsObject() = default;
    explicit PhysicsObject(const glm::vec3& position)
        : m_position(position) {
    }

    const glm::vec3& Position() const { return m_position; }
    void SetPosition(const glm::vec3& p) { m_position = p; m_dirtyTransform = true; }

    const glm::vec3& Velocity() const { return m_velocity; }
    void SetVelocity(const glm::vec3& v) { m_velocity = v; }

    float Mass() const { return m_mass; }
    float InverseMass() const { return m_inverseMass; }

    void SetMass(float mass)
    {
        // mass <= 0 => immovable / infinite mass (common pattern)
        if (mass <= 0.0f)
        {
            m_mass = 0.0f;
            m_inverseMass = 0.0f;
        }
        else
        {
            m_mass = mass;
            m_inverseMass = 1.0f / mass;
        }
    }

    void AddForce(const glm::vec3& f) { m_forceAccum += f; }
    void ClearForces() { m_forceAccum = glm::vec3(0.0f); }

    void Integrate(float dt, IntegratorType type)
    {
        // If inverse mass is 0 => static body
        if (m_inverseMass == 0.0f)
        {
            ClearForces();
            return;
        }

        // a = F * invMass
        const glm::vec3 acceleration = m_forceAccum * m_inverseMass;

        switch (type)
        {
        case IntegratorType::Euler:
            // x += v*dt
            m_position += m_velocity * dt;
            // v += a*dt
            m_velocity += acceleration * dt;
            break;

        case IntegratorType::SemiImplicitEuler:
            // v += a*dt
            m_velocity += acceleration * dt;
            // x += v*dt (uses updated velocity)
            m_position += m_velocity * dt;
            break;
        }

        ClearForces();
        m_dirtyTransform = true;
    }

    const glm::mat4& Transform() const
    {
        if (m_dirtyTransform)
        {
            m_transform = glm::translate(glm::mat4(1.0f), m_position);
            m_dirtyTransform = false;
        }
        return m_transform;
    }

    void ApplyImpulse(const glm::vec3& impulse)
    {
        m_velocity += impulse * m_inverseMass;
    }

private:
    glm::vec3 m_position{ 0.0f };
    glm::vec3 m_velocity{ 0.0f };

    float m_mass{ 1.0f };
    float m_inverseMass{ 1.0f };

    glm::vec3 m_forceAccum{ 0.0f };

    mutable glm::mat4 m_transform{ 1.0f };
    mutable bool m_dirtyTransform{ true };
};