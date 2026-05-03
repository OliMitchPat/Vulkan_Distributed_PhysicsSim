#pragma once
#define NOMINMAX
#include <Windows.h>

#include "World.h"
#include "Components.h"

#include "WorldShapes.h"
#include "BuildWorldShapes.h"   // also pulls in Sphere.h, Plane.h, Capsule.h, Cylinder.h, Cuboid.h
#include "Intersect.h"
#include "Containment.h"
#include "ResolveContact.h"
#include "Transform.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <variant>
#include <vector>
#include <algorithm>
#include <cmath>

class CollisionSystem
{
public:
    void Update(World& world)
    {
        m_contacts.clear();
        m_solids.clear();
        m_containers.clear();

        GatherShapes(world);

        // --- SOLID vs SOLID (Intersect) ---
        TestSolidSolid();

        // --- SOLID vs CONTAINER (Contain) ---
        TestSolidContainer();

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
        RigidBody*  body          = nullptr;
        float restitution         = 0.5f;
        float staticFriction      = 0.5f;
        float dynamicFriction     = 0.3f;
    };

    // Discriminated union of all world shape types, carrying the BodyRef.
    enum class ShapeKind { Sphere, Plane, Capsule, Cylinder, OBB };

    struct AnyShape
    {
        ShapeKind kind;
        BodyRef   br;
        union
        {
            WorldSphere   sphere;
            WorldPlane    plane;
            WorldCapsule  capsule;
            WorldCylinder cylinder;
            WorldOBB      obb;
        };

        AnyShape() {}  // union requires manual construction

        static AnyShape MakeSphere  (const BodyRef& br, const WorldSphere&   ws)
            { AnyShape s; s.kind = ShapeKind::Sphere;   s.br = br; s.sphere   = ws; return s; }
        static AnyShape MakePlane   (const BodyRef& br, const WorldPlane&    wp)
            { AnyShape s; s.kind = ShapeKind::Plane;    s.br = br; s.plane    = wp; return s; }
        static AnyShape MakeCapsule (const BodyRef& br, const WorldCapsule&  wc)
            { AnyShape s; s.kind = ShapeKind::Capsule;  s.br = br; s.capsule  = wc; return s; }
        static AnyShape MakeCylinder(const BodyRef& br, const WorldCylinder& wy)
            { AnyShape s; s.kind = ShapeKind::Cylinder; s.br = br; s.cylinder = wy; return s; }
        static AnyShape MakeOBB     (const BodyRef& br, const WorldOBB&      wo)
            { AnyShape s; s.kind = ShapeKind::OBB;      s.br = br; s.obb      = wo; return s; }
    };

    std::vector<AnyShape> m_solids;
    std::vector<AnyShape> m_containers;

    // Persistent static body used for plane entities without PhysicsComponent.
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

    static float UniformScale(const TransformComponent& tr)
    {
        return std::max({ std::abs(tr.scale.x),
                          std::abs(tr.scale.y),
                          std::abs(tr.scale.z) });
    }

    static Transform BodyTransform(const RigidBody& body, float uniformScale = 1.0f)
    {
        return Transform(body.Position(),
                         glm::mat3_cast(body.Orientation()),
                         uniformScale);
    }

    // Apply a local-center offset rotated by body orientation to world position.
    static glm::vec3 WorldCenter(const RigidBody& body, const glm::vec3& localCenter)
    {
        return body.Position() + glm::mat3_cast(body.Orientation()) * localCenter;
    }

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

    // Push a shape into either solids or containers depending on collision type.
    void AddShape(AnyShape&& shape, CollisionType type)
    {
        if (type == CollisionType::CONTAINER)
            m_containers.push_back(std::move(shape));
        else
            m_solids.push_back(std::move(shape));
    }

    // =========================================================
    // Shape gathering
    // =========================================================

    void GatherShapes(World& world)
    {
        // --- Path 1: ShapeComponent (FlatBuffer scene) ---
        world.forEach<ShapeComponent, PhysicsComponent>(
            [&](Entity /*e*/,
                ShapeComponent& sc,
                PhysicsComponent& phys)
            {
                const Transform physTr = BodyTransform(phys.body);
                const BodyRef   br     = MakeBodyRef(phys.body, &phys);
                const CollisionType ct = sc.collisionType;

                std::visit([&](auto&& shape)
                {
                    using T = std::decay_t<decltype(shape)>;

                    if constexpr (std::is_same_v<T, SphereShape>)
                        AddShape(AnyShape::MakeSphere(br, BuildWorldSphere(physTr, shape)), ct);
                    else if constexpr (std::is_same_v<T, PlaneShape>)
                        AddShape(AnyShape::MakePlane(br, BuildWorldPlane(physTr, shape)), ct);
                    else if constexpr (std::is_same_v<T, CapsuleShape>)
                        AddShape(AnyShape::MakeCapsule(br,
                            BuildWorldCapsule(physTr, Capsule{ shape.radius, shape.height })), ct);
                    else if constexpr (std::is_same_v<T, CylinderShape>)
                        AddShape(AnyShape::MakeCylinder(br,
                            BuildWorldCylinder(physTr, Cylinder{ shape.radius, shape.height })), ct);
                    else if constexpr (std::is_same_v<T, CuboidShape>)
                        AddShape(AnyShape::MakeOBB(br,
                            BuildWorldOBB(physTr, Cuboid{ shape.size })), ct);
                }, sc.shape);
            });

        // --- Path 2: legacy specific-collider components (PrimitiveScene) ---

        // Sphere (always SOLID in legacy scenes)
        world.forEach<SphereColliderComponent, PhysicsComponent>(
            [&](Entity e,
                SphereColliderComponent& sc,
                PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;
                auto* tr = world.getComponent<TransformComponent>(e);
                const float uScale = tr ? UniformScale(*tr) : 1.0f;
                const BodyRef br = MakeBodyRef(phys.body, &phys);

                const glm::vec3 worldCenter = WorldCenter(phys.body, sc.localCenter);
                WorldSphere ws{ worldCenter, sc.baseRadius * uScale };
                m_solids.push_back(AnyShape::MakeSphere(br, ws));
            });

        // Plane with PhysicsComponent (legacy, always SOLID)
        world.forEach<PlaneColliderComponent, PhysicsComponent>(
            [&](Entity e,
                PlaneColliderComponent& pc,
                PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;
                WorldPlane wp{};
                wp.point  = phys.body.Position();
                wp.normal = glm::normalize(glm::mat3_cast(phys.body.Orientation()) * pc.normal);
                m_solids.push_back(AnyShape::MakePlane(MakeBodyRef(phys.body, &phys), wp));
            });

        // Static plane (no PhysicsComponent)
        m_staticBody.SetMotionType(BodyMotionType::Static);
        world.forEach<TransformComponent, PlaneColliderComponent>(
            [&](Entity e, TransformComponent& tr, PlaneColliderComponent& pc)
            {
                if (world.getComponent<PhysicsComponent>(e)) return;
                WorldPlane wp{ tr.position, glm::normalize(pc.normal) };
                m_solids.push_back(AnyShape::MakePlane(MakeBodyRef(m_staticBody, nullptr), wp));
            });

        // Cuboid (legacy, SOLID)
        world.forEach<CuboidColliderComponent, PhysicsComponent>(
            [&](Entity e,
                CuboidColliderComponent& cc,
                PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;
                auto* tr = world.getComponent<TransformComponent>(e);
                const float uScale = tr ? UniformScale(*tr) : 1.0f;
                const BodyRef br = MakeBodyRef(phys.body, &phys);

                // Build a synthetic body at the local-center offset position
                const glm::vec3 centerWorld = WorldCenter(phys.body, cc.localCenter);
                const Transform physTr(centerWorld,
                                       glm::mat3_cast(phys.body.Orientation()),
                                       uScale);
                Cuboid cub{ cc.halfExtents * 2.0f };
                m_solids.push_back(AnyShape::MakeOBB(br, BuildWorldOBB(physTr, cub)));
            });

        // Cylinder (legacy, SOLID)
        world.forEach<CylinderColliderComponent, PhysicsComponent>(
            [&](Entity e,
                CylinderColliderComponent& cc,
                PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;
                auto* tr = world.getComponent<TransformComponent>(e);
                const float uScale = tr ? UniformScale(*tr) : 1.0f;
                const BodyRef br = MakeBodyRef(phys.body, &phys);

                const glm::vec3 centerWorld = WorldCenter(phys.body, cc.localCenter);
                const Transform physTr(centerWorld,
                                       glm::mat3_cast(phys.body.Orientation()),
                                       uScale);
                m_solids.push_back(AnyShape::MakeCylinder(br,
                    BuildWorldCylinder(physTr, Cylinder{ cc.radius, cc.height })));
            });

        // Capsule (legacy, SOLID)
        world.forEach<CapsuleColliderComponent, PhysicsComponent>(
            [&](Entity e,
                CapsuleColliderComponent& cc,
                PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;
                auto* tr = world.getComponent<TransformComponent>(e);
                const float uScale = tr ? UniformScale(*tr) : 1.0f;
                const BodyRef br = MakeBodyRef(phys.body, &phys);

                const glm::vec3 centerWorld = WorldCenter(phys.body, cc.localCenter);
                const Transform physTr(centerWorld,
                                       glm::mat3_cast(phys.body.Orientation()),
                                       uScale);
                m_solids.push_back(AnyShape::MakeCapsule(br,
                    BuildWorldCapsule(physTr, Capsule{ cc.radius, cc.height })));
            });
    }

    // =========================================================
    // SOLID vs SOLID dispatch (uses Intersect)
    // =========================================================

    void TestSolidSolid()
    {
        for (std::size_t i = 0; i < m_solids.size(); ++i)
        {
            for (std::size_t j = i + 1; j < m_solids.size(); ++j)
            {
                PushContact(m_solids[i].br.body, m_solids[j].br.body,
                            IntersectSolids(m_solids[i], m_solids[j]),
                            m_solids[i].br, m_solids[j].br);
            }
        }
    }

    // =========================================================
    // SOLID vs CONTAINER dispatch (uses Contain)
    // =========================================================

    void TestSolidContainer()
    {
        for (auto& solid : m_solids)
        {
            for (auto& container : m_containers)
            {
                // We pass: solid as the "object being contained", container as the "container"
                // The manifold normal from Contain() points inward (push the solid back in).
                // We use solid.br.body as A and container.br.body as B.
                PushContact(solid.br.body, container.br.body,
                            ContainInContainer(solid, container),
                            solid.br, container.br);
            }
        }
    }

    // =========================================================
    // Dispatch helpers
    // =========================================================

    static CollisionManifold IntersectSolids(const AnyShape& a, const AnyShape& b)
    {
        // All 16 pairs available in Intersect.h
        switch (a.kind)
        {
        case ShapeKind::Sphere:
            switch (b.kind)
            {
            case ShapeKind::Sphere:   return Intersect(a.sphere,   b.sphere);
            case ShapeKind::Plane:    return Intersect(a.sphere,   b.plane);
            case ShapeKind::OBB:      return Intersect(a.sphere,   b.obb);
            case ShapeKind::Capsule:  return Intersect(a.sphere,   b.capsule);
            case ShapeKind::Cylinder: return Intersect(a.sphere,   b.cylinder);
            }
            break;
        case ShapeKind::Plane:
            switch (b.kind)
            {
            case ShapeKind::Sphere:   return FlipNormal(Intersect(b.sphere,   a.plane));
            case ShapeKind::OBB:      return FlipNormal(Intersect(b.obb,      a.plane));
            case ShapeKind::Capsule:  return FlipNormal(Intersect(b.capsule,  a.plane));
            case ShapeKind::Cylinder: return FlipNormal(Intersect(b.cylinder, a.plane));
            case ShapeKind::Plane:    return {}; // plane vs plane not meaningful
            }
            break;
        case ShapeKind::OBB:
            switch (b.kind)
            {
            case ShapeKind::Sphere:   return FlipNormal(Intersect(b.sphere,   a.obb));
            case ShapeKind::Plane:    return Intersect(a.obb,      b.plane);
            case ShapeKind::OBB:      return Intersect(a.obb,      b.obb);
            case ShapeKind::Capsule:  return FlipNormal(Intersect(b.capsule,  a.obb));
            case ShapeKind::Cylinder: return FlipNormal(Intersect(b.cylinder, a.obb));
            }
            break;
        case ShapeKind::Capsule:
            switch (b.kind)
            {
            case ShapeKind::Sphere:   return FlipNormal(Intersect(b.sphere,   a.capsule));
            case ShapeKind::Plane:    return Intersect(a.capsule,  b.plane);
            case ShapeKind::OBB:      return Intersect(a.capsule,  b.obb);
            case ShapeKind::Capsule:  return Intersect(a.capsule,  b.capsule);
            case ShapeKind::Cylinder: return Intersect(a.capsule,  b.cylinder);
            }
            break;
        case ShapeKind::Cylinder:
            switch (b.kind)
            {
            case ShapeKind::Sphere:   return FlipNormal(Intersect(b.sphere,   a.cylinder));
            case ShapeKind::Plane:    return Intersect(a.cylinder, b.plane);
            case ShapeKind::OBB:      return Intersect(a.cylinder, b.obb);
            case ShapeKind::Capsule:  return FlipNormal(Intersect(b.capsule,  a.cylinder));
            case ShapeKind::Cylinder: return Intersect(a.cylinder, b.cylinder);
            }
            break;
        }
        return {};
    }

    // Dispatch solid vs container using Contain().
    // The 'inner' is the solid (must stay inside), 'outer' is the container.
    static CollisionManifold ContainInContainer(const AnyShape& inner, const AnyShape& outer)
    {
        switch (outer.kind)
        {
        case ShapeKind::Sphere:
            switch (inner.kind)
            {
            case ShapeKind::Sphere:   return Contain(inner.sphere,   outer.sphere);
            case ShapeKind::Capsule:  return Contain(inner.capsule,  outer.sphere);
            case ShapeKind::Cylinder: return Contain(inner.cylinder, outer.sphere);
            case ShapeKind::OBB:      return Contain(inner.obb,      outer.sphere);
            default: break;
            }
            break;
        case ShapeKind::OBB:
            switch (inner.kind)
            {
            case ShapeKind::Sphere:   return Contain(inner.sphere,   outer.obb);
            case ShapeKind::Capsule:  return Contain(inner.capsule,  outer.obb);
            case ShapeKind::Cylinder: return Contain(inner.cylinder, outer.obb);
            case ShapeKind::OBB:      return Contain(inner.obb,      outer.obb);
            default: break;
            }
            break;
        case ShapeKind::Cylinder:
            switch (inner.kind)
            {
            case ShapeKind::Sphere:   return Contain(inner.sphere,   outer.cylinder);
            case ShapeKind::Capsule:  return Contain(inner.capsule,  outer.cylinder);
            case ShapeKind::Cylinder: return Contain(inner.cylinder, outer.cylinder);
            case ShapeKind::OBB:      return Contain(inner.obb,      outer.cylinder);
            default: break;
            }
            break;
        case ShapeKind::Plane:
            // Plane container = half-space; keep solid on the negative-normal side.
            switch (inner.kind)
            {
            case ShapeKind::Sphere:   return Contain(inner.sphere,   outer.plane);
            case ShapeKind::Capsule:  return Contain(inner.capsule,  outer.plane);
            case ShapeKind::Cylinder: return Contain(inner.cylinder, outer.plane);
            case ShapeKind::OBB:      return Contain(inner.obb,      outer.plane);
            default: break;
            }
            break;
        default:
            break;
        }
        return {};
    }

    // Flip normal and contact-point so "A/B" assignment stays consistent
    // after we used a reversed argument order to Intersect.
    static CollisionManifold FlipNormal(CollisionManifold m)
    {
        if (m.hit) m.normal = -m.normal;
        return m;
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
