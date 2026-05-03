#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

enum class IntegratorType
{
    Euler,
    SemiImplicitEuler,
};

enum class BodyMotionType
{
    Dynamic,   // simulated: impulses + integration
    Static,    // immovable: no impulses, no integration, velocities treated as zero
    Kinematic  // moved externally: no impulses, no integration, BUT velocities are user-provided
};

class RigidBody
{
public:
    // ---- Motion type ----
    BodyMotionType MotionType() const { return m_motionType; }
    void SetMotionType(BodyMotionType t)
    {
        m_motionType = t;

        if (m_motionType == BodyMotionType::Static)
        {
            m_linearVelocity = glm::vec3(0.0f);
            m_angularVelocity = glm::vec3(0.0f);
            m_invMass = 0.0f;
            m_mass = 0.0f;
            m_invInertiaBody = glm::mat3(0.0f);
            m_invInertiaWorld = glm::mat3(0.0f);
        }
        else if (m_motionType == BodyMotionType::Kinematic)
        {
            m_invMass = 0.0f;
            m_mass = 0.0f;
            m_invInertiaBody = glm::mat3(0.0f);
            m_invInertiaWorld = glm::mat3(0.0f);
        }
    }

    bool IsDynamic() const { return m_motionType == BodyMotionType::Dynamic; }
    bool IsStatic() const { return m_motionType == BodyMotionType::Static; }
    bool IsKinematic() const { return m_motionType == BodyMotionType::Kinematic; }

    // ----- Pose -----
    const glm::vec3& Position() const { return m_position; }
    void SetPosition(const glm::vec3& p) { m_position = p; m_dirtyTransform = true; }

    const glm::quat& Orientation() const { return m_orientation; }
    void SetOrientation(const glm::quat& q)
    {
        m_orientation = glm::normalize(q);
        m_dirtyTransform = true;
        UpdateInertiaWorld();
    }

    // ----- Velocities -----
    const glm::vec3& LinearVelocity() const { return m_linearVelocity; }
    void SetLinearVelocity(const glm::vec3& v) { m_linearVelocity = v; }

    const glm::vec3& AngularVelocity() const { return m_angularVelocity; }
    void SetAngularVelocity(const glm::vec3& wRad) { m_angularVelocity = wRad; }

    // ----- Damping -----
    // These are in 1/seconds. 0 = no damping. Typical small values: 0.01..0.2
    float LinearDamping() const { return m_linearDamping; }
    float AngularDamping() const { return m_angularDamping; }

    void SetLinearDamping(float d) { m_linearDamping = std::max(0.0f, d); }
    void SetAngularDamping(float d) { m_angularDamping = std::max(0.0f, d); }

    // ----- Mass -----
    float Mass() const { return m_mass; }
    float InverseMass() const { return m_invMass; }

    void SetMass(float mass)
    {
        if (mass <= 0.0f)
        {
            m_mass = 0.0f;
            m_invMass = 0.0f;
        }
        else
        {
            m_mass = mass;
            m_invMass = 1.0f / mass;
        }

        if (!IsDynamic())
        {
            m_mass = 0.0f;
            m_invMass = 0.0f;
        }
    }

    // ----- Inertia -----
    void SetInverseInertiaBody(const glm::mat3& invIb)
    {
        if (!IsDynamic())
        {
            m_invInertiaBody = glm::mat3(0.0f);
            m_invInertiaWorld = glm::mat3(0.0f);
            return;
        }

        m_invInertiaBody = invIb;
        UpdateInertiaWorld();
    }

    const glm::mat3& InverseInertiaBody() const { return m_invInertiaBody; }
    const glm::mat3& InverseInertiaWorld() const { return m_invInertiaWorld; }

    float EffectiveInverseMass() const { return IsDynamic() ? m_invMass : 0.0f; }
    glm::mat3 EffectiveInverseInertiaWorld() const { return IsDynamic() ? m_invInertiaWorld : glm::mat3(0.0f); }

    // ----- Forces / Torques -----
    void AddForce(const glm::vec3& f) { if (IsDynamic()) m_forceAccum += f; }
    void AddTorque(const glm::vec3& t) { if (IsDynamic()) m_torqueAccum += t; }
    void ClearAccumulators() { m_forceAccum = {}; m_torqueAccum = {}; }

    // ----- Integration -----
    void Integrate(float dt, IntegratorType type)
    {
        if (!IsDynamic())
        {
            ClearAccumulators();
            return;
        }

        // Linear
        const glm::vec3 a = m_forceAccum * m_invMass;

        if (type == IntegratorType::Euler)
        {
            m_position += m_linearVelocity * dt;
            m_linearVelocity += a * dt;
        }
        else
        {
            m_linearVelocity += a * dt;
            m_position += m_linearVelocity * dt;
        }

        // Angular
        const glm::vec3 wDot = m_invInertiaWorld * m_torqueAccum;
        m_angularVelocity += wDot * dt;

        // --- Damping (stable, dt-aware) ---
        // v *= 1 / (1 + d*dt) avoids frame-rate dependence and is unconditionally stable.
        if (m_linearDamping > 0.0f)
            m_linearVelocity *= 1.0f / (1.0f + m_linearDamping * dt);

        if (m_angularDamping > 0.0f)
            m_angularVelocity *= 1.0f / (1.0f + m_angularDamping * dt);

        // dq/dt = 0.5 * omegaQuat * q
        const glm::quat omega(0.0f, m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z);
        m_orientation = glm::normalize(m_orientation + (0.5f * omega * m_orientation) * dt);

        UpdateInertiaWorld();
        ClearAccumulators();
        m_dirtyTransform = true;
    }

    // ----- Impulses -----
    void ApplyImpulse(const glm::vec3& impulse)
    {
        if (!IsDynamic()) return;
        m_linearVelocity += impulse * m_invMass;
    }

    void ApplyImpulseAtPoint(const glm::vec3& impulse, const glm::vec3& worldPoint)
    {
        if (!IsDynamic()) return;

        m_linearVelocity += impulse * m_invMass;

        const glm::vec3 r = worldPoint - m_position;
        m_angularVelocity += m_invInertiaWorld * glm::cross(r, impulse);
    }

    glm::mat4 Transform() const
    {
        if (m_dirtyTransform)
        {
            const glm::mat4 T = glm::translate(glm::mat4(1.0f), m_position);
            const glm::mat4 R = glm::mat4_cast(m_orientation);
            m_transform = T * R;
            m_dirtyTransform = false;
        }
        return m_transform;
    }

private:
    void UpdateInertiaWorld()
    {
        if (!IsDynamic())
        {
            m_invInertiaWorld = glm::mat3(0.0f);
            return;
        }

        const glm::mat3 R = glm::mat3_cast(m_orientation);
        m_invInertiaWorld = R * m_invInertiaBody * glm::transpose(R);
    }

private:
    BodyMotionType m_motionType{ BodyMotionType::Dynamic };

    glm::vec3 m_position{ 0.0f };
    glm::quat m_orientation{ 1.0f, 0.0f, 0.0f, 0.0f };

    glm::vec3 m_linearVelocity{ 0.0f };
    glm::vec3 m_angularVelocity{ 0.0f };

    float m_mass{ 1.0f };
    float m_invMass{ 1.0f };

    glm::mat3 m_invInertiaBody{ 1.0f };
    glm::mat3 m_invInertiaWorld{ 1.0f };

    glm::vec3 m_forceAccum{ 0.0f };
    glm::vec3 m_torqueAccum{ 0.0f };

    // Damping coefficients (1/sec)
    float m_linearDamping{ 0.02f };
    float m_angularDamping{ 0.05f };

    mutable glm::mat4 m_transform{ 1.0f };
    mutable bool m_dirtyTransform{ true };
};