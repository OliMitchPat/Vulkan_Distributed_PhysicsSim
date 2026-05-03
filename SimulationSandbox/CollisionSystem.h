#pragma once
#define NOMINMAX
#include <Windows.h>

#include "World.h"
#include "Components.h"

#include "WorldShapes.h"
#include "BuildWorldShapes.h"
#include "Intersect.h"
#include "ResolveContact.h"
#include "Transform.h"
#include "Sphere.h"
#include "Plane.h"
#include "Capsule.h"
#include "Cylinder.h"
#include "Cuboid.h"
#include "ShapeData.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

class CollisionSystem
{
public:
    void Update(World& world)
    {
        m_contacts.clear();
        m_spheres.clear();
        m_planes.clear();
        m_capsules.clear();
        m_cylinders.clear();
        m_obbs.clear();

        GatherShapes(world);

        // --- Sphere pairs ---
        TestSphereSphere();
        TestSpherePlane();
        TestSphereOBB();
        TestSphereCapsule();
        TestSphereCylinder();

        // --- Other vs Plane ---
        TestOBBPlane();
        TestCapsulePlane();
        TestCylinderPlane();

        // --- Dynamic-dynamic non-sphere demo ---
        TestOBBOBB();
        TestCapsuleCapsule();
        TestCapsuleOBB();
        TestCapsuleCylinder();
        TestCylinderOBB();
        TestCylinderCylinder();

        // Solve contacts (velocity impulses)
        const int iterations = 8;
        for (int i = 0; i < iterations; ++i)
            for (auto& c : m_contacts)
                SolveContactImpulse(*c.A, *c.B, c.manifold, c.material);

        // Positional correction
        for (auto& c : m_contacts)
            PositionalCorrection(*c.A, *c.B, c.manifold);
    }

private:
    // ---- Per-body metadata carried alongside each world shape ----
    struct BodyRef
    {
        RigidBody*  body        = nullptr;
        float restitution       = 0.5f;
        float staticFriction    = 0.5f;
        float dynamicFriction   = 0.3f;
    };

    struct SphereEntry   { BodyRef br; WorldSphere   ws; };
    struct PlaneEntry    { BodyRef br; WorldPlane    wp; };
    struct CapsuleEntry  { BodyRef br; WorldCapsule  wc; };
    struct CylinderEntry { BodyRef br; WorldCylinder wy; };
    struct OBBEntry      { BodyRef br; WorldOBB      wo; };

    std::vector<SphereEntry>   m_spheres;
    std::vector<PlaneEntry>    m_planes;
    std::vector<CapsuleEntry>  m_capsules;
    std::vector<CylinderEntry> m_cylinders;
    std::vector<OBBEntry>      m_obbs;

    // Persistent static body used for plane entities that have no PhysicsComponent.
    RigidBody m_staticBody;

    // ---- Contact cache ----
    struct Contact
    {
        RigidBody*       A;
        RigidBody*       B;
        CollisionManifold manifold;
        ContactMaterial   material;
    };
    std::vector<Contact> m_contacts;

    // =========================================================
    // Helpers
    // =========================================================

    // Max-abs uniform scale (Option A from spec).
    static float UniformScale(const TransformComponent& tr)
    {
        return std::max({ std::abs(tr.scale.x),
                          std::abs(tr.scale.y),
                          std::abs(tr.scale.z) });
    }

    // Build a StaticLib Transform from body pose (used for ShapeComponent path).
    // Shape dimensions are already world-scaled, so uniformScale = 1.
    static Transform BodyTransform(const RigidBody& body)
    {
        return Transform(body.Position(),
                         glm::mat3_cast(body.Orientation()),
                         1.0f);
    }

    // Build a StaticLib Transform with explicit uniform scale (legacy collider path).
    static Transform BodyTransformScaled(const RigidBody& body, float uniformScale)
    {
        return Transform(body.Position(),
                         glm::mat3_cast(body.Orientation()),
                         uniformScale);
    }

    // Combine materials from two BodyRefs into a ContactMaterial.
    static ContactMaterial CombineMaterial(const BodyRef& a, const BodyRef& b)
    {
        ContactMaterial mat{};
        mat.restitution    = std::min(a.restitution,    b.restitution);
        mat.staticFriction  = (a.staticFriction  + b.staticFriction)  * 0.5f;
        mat.dynamicFriction = (a.dynamicFriction + b.dynamicFriction) * 0.5f;
        return mat;
    }

    void PushContact(RigidBody* A, RigidBody* B,
                     const CollisionManifold& m,
                     const BodyRef& brA, const BodyRef& brB)
    {
        if (!m.hit) return;
        m_contacts.push_back({ A, B, m, CombineMaterial(brA, brB) });
    }

    // =========================================================
    // Shape gathering
    // =========================================================

    // Fill BodyRef from a PhysicsComponent (or use static defaults).
    static BodyRef MakeBodyRef(RigidBody& body, PhysicsComponent* phys)
    {
        BodyRef br{};
        br.body = &body;
        if (phys)
        {
            br.restitution    = phys->restitution;
            br.staticFriction  = phys->staticFriction;
            br.dynamicFriction = phys->dynamicFriction;
        }
        return br;
    }

    void GatherShapes(World& world)
    {
        // --- Path 1: ShapeComponent (FlatBuffer scene) ---
        // Shape dimensions are already multiplied by uniformScale in ScaleShape().
        world.forEach<ShapeComponent, PhysicsComponent>(
            [&](Entity /*e*/,
                ShapeComponent& sc,
                PhysicsComponent& phys)
            {
                const Transform physTr = BodyTransform(phys.body);
                const BodyRef   br     = MakeBodyRef(phys.body, &phys);

                std::visit([&](auto&& shape)
                {
                    using T = std::decay_t<decltype(shape)>;

                    if constexpr (std::is_same_v<T, SphereShape>)
                    {
                        m_spheres.push_back({ br,
                            BuildWorldSphere(physTr, shape) });
                    }
                    else if constexpr (std::is_same_v<T, PlaneShape>)
                    {
                        m_planes.push_back({ br,
                            BuildWorldPlane(physTr, shape) });
                    }
                    else if constexpr (std::is_same_v<T, CapsuleShape>)
                    {
                        m_capsules.push_back({ br,
                            BuildWorldCapsule(physTr, Capsule{ shape.radius, shape.height }) });
                    }
                    else if constexpr (std::is_same_v<T, CylinderShape>)
                    {
                        m_cylinders.push_back({ br,
                            BuildWorldCylinder(physTr, Cylinder{ shape.radius, shape.height }) });
                    }
                    else if constexpr (std::is_same_v<T, CuboidShape>)
                    {
                        m_obbs.push_back({ br,
                            BuildWorldOBB(physTr, Cuboid{ shape.size }) });
                    }
                }, sc.shape);
            });

        // --- Path 2: legacy specific-collider components (PrimitiveScene) ---

        // Sphere
        world.forEach<SphereColliderComponent, PhysicsComponent>(
            [&](Entity e,
                SphereColliderComponent& sc,
                PhysicsComponent& phys)
            {
                // Skip if already handled via ShapeComponent
                if (world.getComponent<ShapeComponent>(e)) return;

                auto* tr = world.getComponent<TransformComponent>(e);
                const float uScale = tr ? UniformScale(*tr) : 1.0f;
                const BodyRef br = MakeBodyRef(phys.body, &phys);

                // Apply localCenter offset rotated by body orientation
                const glm::vec3 worldCenter =
                    phys.body.Position() +
                    glm::mat3_cast(phys.body.Orientation()) * sc.localCenter;

                WorldSphere ws{};
                ws.center = worldCenter;
                ws.radius  = sc.baseRadius * uScale;
                m_spheres.push_back({ br, ws });
            });

        // Plane with PhysicsComponent (legacy dynamic planes, unlikely but handled)
        world.forEach<PlaneColliderComponent, PhysicsComponent>(
            [&](Entity e,
                PlaneColliderComponent& pc,
                PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;
                WorldPlane wp{};
                wp.point  = phys.body.Position();
                wp.normal = glm::normalize(
                    glm::mat3_cast(phys.body.Orientation()) * pc.normal);
                const BodyRef br = MakeBodyRef(phys.body, &phys);
                m_planes.push_back({ br, wp });
            });

        // Static plane (no PhysicsComponent) — most common in PrimitiveScene
        m_staticBody.SetMotionType(BodyMotionType::Static);
        world.forEach<TransformComponent, PlaneColliderComponent>(
            [&](Entity e, TransformComponent& tr, PlaneColliderComponent& pc)
            {
                // Only add once per entity — skip if this entity also has PhysicsComponent
                if (world.getComponent<PhysicsComponent>(e)) return;

                WorldPlane wp{};
                wp.point  = tr.position;
                wp.normal = glm::normalize(pc.normal);

                BodyRef br = MakeBodyRef(m_staticBody, nullptr);
                m_planes.push_back({ br, wp });
            });

        // Cuboid (legacy)
        world.forEach<CuboidColliderComponent, PhysicsComponent>(
            [&](Entity e,
                CuboidColliderComponent& cc,
                PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;
                auto* tr = world.getComponent<TransformComponent>(e);
                const float uScale = tr ? UniformScale(*tr) : 1.0f;
                const Transform physTr = BodyTransformScaled(phys.body, uScale);
                const BodyRef   br     = MakeBodyRef(phys.body, &phys);

                // halfExtents -> full size for Cuboid
                Cuboid cub{ cc.halfExtents * 2.0f };
                m_obbs.push_back({ br, BuildWorldOBB(physTr, cub) });
            });

        // Cylinder (legacy)
        world.forEach<CylinderColliderComponent, PhysicsComponent>(
            [&](Entity e,
                CylinderColliderComponent& cc,
                PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;
                auto* tr = world.getComponent<TransformComponent>(e);
                const float uScale = tr ? UniformScale(*tr) : 1.0f;
                const Transform physTr = BodyTransformScaled(phys.body, uScale);
                const BodyRef   br     = MakeBodyRef(phys.body, &phys);

                Cylinder cyl{ cc.radius, cc.height };
                m_cylinders.push_back({ br, BuildWorldCylinder(physTr, cyl) });
            });

        // Capsule (legacy)
        world.forEach<CapsuleColliderComponent, PhysicsComponent>(
            [&](Entity e,
                CapsuleColliderComponent& cc,
                PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;
                auto* tr = world.getComponent<TransformComponent>(e);
                const float uScale = tr ? UniformScale(*tr) : 1.0f;
                const Transform physTr = BodyTransformScaled(phys.body, uScale);
                const BodyRef   br     = MakeBodyRef(phys.body, &phys);

                Capsule cap{ cc.radius, cc.height };
                m_capsules.push_back({ br, BuildWorldCapsule(physTr, cap) });
            });
    }

    // =========================================================
    // Collision pair tests
    // =========================================================

    void TestSphereSphere()
    {
        for (std::size_t i = 0; i < m_spheres.size(); ++i)
            for (std::size_t j = i + 1; j < m_spheres.size(); ++j)
                PushContact(m_spheres[i].br.body, m_spheres[j].br.body,
                            Intersect(m_spheres[i].ws, m_spheres[j].ws),
                            m_spheres[i].br, m_spheres[j].br);
    }

    void TestSpherePlane()
    {
        for (auto& se : m_spheres)
            for (auto& pe : m_planes)
                PushContact(se.br.body, pe.br.body,
                            Intersect(se.ws, pe.wp),
                            se.br, pe.br);
    }

    void TestSphereOBB()
    {
        for (auto& se : m_spheres)
            for (auto& oe : m_obbs)
                PushContact(se.br.body, oe.br.body,
                            Intersect(se.ws, oe.wo),
                            se.br, oe.br);
    }

    void TestSphereCapsule()
    {
        for (auto& se : m_spheres)
            for (auto& ce : m_capsules)
                PushContact(se.br.body, ce.br.body,
                            Intersect(se.ws, ce.wc),
                            se.br, ce.br);
    }

    void TestSphereCylinder()
    {
        for (auto& se : m_spheres)
            for (auto& ye : m_cylinders)
                PushContact(se.br.body, ye.br.body,
                            Intersect(se.ws, ye.wy),
                            se.br, ye.br);
    }

    void TestOBBPlane()
    {
        for (auto& oe : m_obbs)
            for (auto& pe : m_planes)
                PushContact(oe.br.body, pe.br.body,
                            Intersect(oe.wo, pe.wp),
                            oe.br, pe.br);
    }

    void TestCapsulePlane()
    {
        for (auto& ce : m_capsules)
            for (auto& pe : m_planes)
                PushContact(ce.br.body, pe.br.body,
                            Intersect(ce.wc, pe.wp),
                            ce.br, pe.br);
    }

    void TestCylinderPlane()
    {
        for (auto& ye : m_cylinders)
            for (auto& pe : m_planes)
                PushContact(ye.br.body, pe.br.body,
                            Intersect(ye.wy, pe.wp),
                            ye.br, pe.br);
    }

    void TestOBBOBB()
    {
        for (std::size_t i = 0; i < m_obbs.size(); ++i)
            for (std::size_t j = i + 1; j < m_obbs.size(); ++j)
                PushContact(m_obbs[i].br.body, m_obbs[j].br.body,
                            Intersect(m_obbs[i].wo, m_obbs[j].wo),
                            m_obbs[i].br, m_obbs[j].br);
    }

    void TestCapsuleCapsule()
    {
        for (std::size_t i = 0; i < m_capsules.size(); ++i)
            for (std::size_t j = i + 1; j < m_capsules.size(); ++j)
                PushContact(m_capsules[i].br.body, m_capsules[j].br.body,
                            Intersect(m_capsules[i].wc, m_capsules[j].wc),
                            m_capsules[i].br, m_capsules[j].br);
    }

    void TestCapsuleOBB()
    {
        for (auto& ce : m_capsules)
            for (auto& oe : m_obbs)
                PushContact(ce.br.body, oe.br.body,
                            Intersect(ce.wc, oe.wo),
                            ce.br, oe.br);
    }

    void TestCapsuleCylinder()
    {
        for (auto& ce : m_capsules)
            for (auto& ye : m_cylinders)
                PushContact(ce.br.body, ye.br.body,
                            Intersect(ce.wc, ye.wy),
                            ce.br, ye.br);
    }

    void TestCylinderOBB()
    {
        for (auto& ye : m_cylinders)
            for (auto& oe : m_obbs)
                PushContact(ye.br.body, oe.br.body,
                            Intersect(ye.wy, oe.wo),
                            ye.br, oe.br);
    }

    void TestCylinderCylinder()
    {
        for (std::size_t i = 0; i < m_cylinders.size(); ++i)
            for (std::size_t j = i + 1; j < m_cylinders.size(); ++j)
                PushContact(m_cylinders[i].br.body, m_cylinders[j].br.body,
                            Intersect(m_cylinders[i].wy, m_cylinders[j].wy),
                            m_cylinders[i].br, m_cylinders[j].br);
    }

    // =========================================================
    // Positional correction (Baumgarte-style)
    // =========================================================

    static void PositionalCorrection(RigidBody& A, RigidBody& B, const CollisionManifold& m)
    {
        const float percent = 0.8f;
        const float slop    = 0.01f;

        const float invMassA = A.EffectiveInverseMass();
        const float invMassB = B.EffectiveInverseMass();
        const float totalInvMass = invMassA + invMassB;
        if (totalInvMass <= 0.0f) return;

        const float corrMag  = std::max(m.penetration - slop, 0.0f) / totalInvMass;
        const glm::vec3 corr = percent * corrMag * m.normal;

        if (A.IsDynamic()) A.SetPosition(A.Position() - corr * invMassA);
        if (B.IsDynamic()) B.SetPosition(B.Position() + corr * invMassB);
    }
};