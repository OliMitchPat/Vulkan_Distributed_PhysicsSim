#pragma once
#define NOMINMAX
#include <Windows.h>

#include "World.h"
#include "Components.h"

#include "WorldShapes.h"
#include "BuildWorldShapes.h"
#include "Intersect.h"
#include "Containment.h"
#include "ResolveContact.h"

#include <cstdio>
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

        TestSolidSolid();
        TestSolidContainer();

        const int iterations = 8;
        for (int i = 0; i < iterations; ++i)
        {
            for (auto& c : m_contacts)
            {
                ResolveContact(*c.A, *c.B, c.manifold, c.material);
            }
        }

        // Critical: sync ECS transform from physics bodies after collision response
        world.forEach<TransformComponent, PhysicsComponent>(
            [&](Entity, TransformComponent& tr, PhysicsComponent& phys)
            {
                tr.position = phys.body.Position();
            });
    }

private:
    struct BodyRef
    {
        RigidBody* body = nullptr;
        float restitution = 0.5f;
        float staticFriction = 0.5f;
        float dynamicFriction = 0.3f;
    };

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

        AnyShape() {}

        static AnyShape MakeSphere(const BodyRef& br, const WorldSphere& ws)
        {
            AnyShape s; s.kind = ShapeKind::Sphere; s.br = br; s.sphere = ws; return s;
        }

        static AnyShape MakePlane(const BodyRef& br, const WorldPlane& wp)
        {
            AnyShape s; s.kind = ShapeKind::Plane; s.br = br; s.plane = wp; return s;
        }

        static AnyShape MakeCapsule(const BodyRef& br, const WorldCapsule& wc)
        {
            AnyShape s; s.kind = ShapeKind::Capsule; s.br = br; s.capsule = wc; return s;
        }

        static AnyShape MakeCylinder(const BodyRef& br, const WorldCylinder& wy)
        {
            AnyShape s; s.kind = ShapeKind::Cylinder; s.br = br; s.cylinder = wy; return s;
        }

        static AnyShape MakeOBB(const BodyRef& br, const WorldOBB& wo)
        {
            AnyShape s; s.kind = ShapeKind::OBB; s.br = br; s.obb = wo; return s;
        }
    };

    std::vector<AnyShape> m_solids;
    std::vector<AnyShape> m_containers;

    struct Contact
    {
        RigidBody* A;
        RigidBody* B;
        CollisionManifold manifold;
        ContactMaterial   material;
    };
    std::vector<Contact> m_contacts;

    // =========================================================
    // Helpers
    // =========================================================

    static float UniformScale(const TransformComponent* tr)
    {
        if (!tr) return 1.0f;
        return std::max({ std::abs(tr->scale.x), std::abs(tr->scale.y), std::abs(tr->scale.z) });
    }

    static Transform BodyTransform(const RigidBody& body, float uniformScale)
    {
        return Transform(body.Position(), glm::mat3_cast(body.Orientation()), uniformScale);
    }

    static ContactMaterial CombineMaterial(const BodyRef& a, const BodyRef& b)
    {
        ContactMaterial mat{};
        mat.restitution = std::min(a.restitution, b.restitution);
        mat.staticFriction = 0.5f * (a.staticFriction + b.staticFriction);
        mat.dynamicFriction = 0.5f * (a.dynamicFriction + b.dynamicFriction);
        return mat;
    }

    void PushContact(const AnyShape& a, const AnyShape& b, const CollisionManifold& m)
    {
        if (!m.hit) return;

        // DEBUG: only print for Sphere vs Plane to reduce spam
        if ((a.kind == ShapeKind::Sphere && b.kind == ShapeKind::Plane) ||
            (a.kind == ShapeKind::Plane && b.kind == ShapeKind::Sphere))
        {
            printf("[Contact] %s-%s pen=%.6f n=(%.3f,%.3f,%.3f) p=(%.3f,%.3f,%.3f)\n",
                (a.kind == ShapeKind::Sphere ? "Sphere" : "Plane"),
                (b.kind == ShapeKind::Plane ? "Plane" : "Sphere"),
                m.penetration,
                m.normal.x, m.normal.y, m.normal.z,
                m.contactPoint.x, m.contactPoint.y, m.contactPoint.z);
        }

        m_contacts.push_back({ a.br.body, b.br.body, m, CombineMaterial(a.br, b.br) });
    }

    static BodyRef MakeBodyRef(PhysicsComponent& phys)
    {
        BodyRef br{};
        br.body = &phys.body;
        br.restitution = phys.restitution;
        br.staticFriction = phys.staticFriction;
        br.dynamicFriction = phys.dynamicFriction;
        return br;
    }

    void AddShape(AnyShape&& shape, CollisionType type)
    {
        if (type == CollisionType::CONTAINER) m_containers.push_back(std::move(shape));
        else m_solids.push_back(std::move(shape));
    }

    static CollisionManifold FlipNormal(CollisionManifold m)
    {
        if (m.hit) m.normal = -m.normal;
        return m;
    }

    // Ensure manifold normal points from A -> B (solver convention).
    static CollisionManifold ForceNormalAtoB(CollisionManifold m, const glm::vec3& posA, const glm::vec3& posB)
    {
        if (!m.hit) return m;
        const glm::vec3 ab = posB - posA;
        if (glm::dot(m.normal, ab) < 0.0f)
            m.normal = -m.normal;
        return m;
    }

    static glm::vec3 CapsuleCenter(const WorldCapsule& c) { return 0.5f * (c.a + c.b); }
    static glm::vec3 CylinderCenter(const WorldCylinder& c) { return 0.5f * (c.a + c.b); }

    // =========================================================
    // Shape gathering (pose from RigidBody; scale optionally from TransformComponent)
    // =========================================================

    void GatherShapes(World& world)
    {
        // ShapeComponent path (FlatBuffer scenes)
        world.forEach<ShapeComponent, PhysicsComponent>(
            [&](Entity e, ShapeComponent& sc, PhysicsComponent& phys)
            {
                TransformComponent* tr = world.getComponent<TransformComponent>(e);
                const float uScale = UniformScale(tr);

                const Transform physTr = BodyTransform(phys.body, uScale);
                const BodyRef br = MakeBodyRef(phys);
                const CollisionType ct = sc.collisionType;

                std::visit([&](auto&& shape)
                    {
                        using T = std::decay_t<decltype(shape)>;

                        if constexpr (std::is_same_v<T, SphereShape>)
                            AddShape(AnyShape::MakeSphere(br, BuildWorldSphere(physTr, shape)), ct);
                        else if constexpr (std::is_same_v<T, PlaneShape>)
                            AddShape(AnyShape::MakePlane(br, BuildWorldPlane(physTr, shape)), ct);
                        else if constexpr (std::is_same_v<T, CapsuleShape>)
                            AddShape(AnyShape::MakeCapsule(br, BuildWorldCapsule(physTr, shape)), ct);
                        else if constexpr (std::is_same_v<T, CylinderShape>)
                            AddShape(AnyShape::MakeCylinder(br, BuildWorldCylinder(physTr, shape)), ct);
                        else if constexpr (std::is_same_v<T, CuboidShape>)
                            AddShape(AnyShape::MakeOBB(br, BuildWorldOBB(physTr, shape)), ct);
                    }, sc.shape);
            });

        // Legacy sphere collider
        world.forEach<SphereColliderComponent, PhysicsComponent>(
            [&](Entity e, SphereColliderComponent& sc, PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;

                TransformComponent* tr = world.getComponent<TransformComponent>(e);
                const float uScale = UniformScale(tr);

                const BodyRef br = MakeBodyRef(phys);

                Transform t = BodyTransform(phys.body, uScale);

                SphereShape ss{};
                ss.radius = sc.baseRadius;
                WorldSphere ws = BuildWorldSphere(t, ss);

                ws.center += glm::mat3_cast(phys.body.Orientation()) * sc.localCenter;

                m_solids.push_back(AnyShape::MakeSphere(br, ws));
            });

        // Legacy plane collider (with PhysicsComponent)
        world.forEach<PlaneColliderComponent, PhysicsComponent>(
            [&](Entity e, PlaneColliderComponent& pc, PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;

                const BodyRef br = MakeBodyRef(phys);

                WorldPlane wp{};
                wp.point = phys.body.Position();
                wp.normal = glm::normalize(pc.normal); // keep unrotated for now

                m_solids.push_back(AnyShape::MakePlane(br, wp));
            });

        // Legacy cuboid collider
        world.forEach<CuboidColliderComponent, PhysicsComponent>(
            [&](Entity e, CuboidColliderComponent& cc, PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;

                TransformComponent* tr = world.getComponent<TransformComponent>(e);
                const float uScale = UniformScale(tr);

                const BodyRef br = MakeBodyRef(phys);

                Transform t = BodyTransform(phys.body, uScale);
                t.position += glm::mat3_cast(phys.body.Orientation()) * cc.localCenter;

                CuboidShape cub{};
                cub.size = cc.halfExtents * 2.0f;

                m_solids.push_back(AnyShape::MakeOBB(br, BuildWorldOBB(t, cub)));
            });

        // Legacy cylinder collider
        world.forEach<CylinderColliderComponent, PhysicsComponent>(
            [&](Entity e, CylinderColliderComponent& cc, PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;

                TransformComponent* tr = world.getComponent<TransformComponent>(e);
                const float uScale = UniformScale(tr);

                const BodyRef br = MakeBodyRef(phys);

                Transform t = BodyTransform(phys.body, uScale);
                t.position += glm::mat3_cast(phys.body.Orientation()) * cc.localCenter;

                CylinderShape cy{};
                cy.radius = cc.radius;
                cy.height = cc.height;

                m_solids.push_back(AnyShape::MakeCylinder(br, BuildWorldCylinder(t, cy)));
            });

        // Legacy capsule collider
        world.forEach<CapsuleColliderComponent, PhysicsComponent>(
            [&](Entity e, CapsuleColliderComponent& cc, PhysicsComponent& phys)
            {
                if (world.getComponent<ShapeComponent>(e)) return;

                TransformComponent* tr = world.getComponent<TransformComponent>(e);
                const float uScale = UniformScale(tr);

                const BodyRef br = MakeBodyRef(phys);

                Transform t = BodyTransform(phys.body, uScale);
                t.position += glm::mat3_cast(phys.body.Orientation()) * cc.localCenter;

                CapsuleShape cap{};
                cap.radius = cc.radius;
                cap.height = cc.height;

                m_solids.push_back(AnyShape::MakeCapsule(br, BuildWorldCapsule(t, cap)));
            });
    }

    // =========================================================
    // Dispatch
    // =========================================================

    void TestSolidSolid()
    {
        for (std::size_t i = 0; i < m_solids.size(); ++i)
        {
            for (std::size_t j = i + 1; j < m_solids.size(); ++j)
            {
                PushContact(m_solids[i], m_solids[j], IntersectSolids(m_solids[i], m_solids[j]));
            }
        }
    }

    void TestSolidContainer()
    {
        for (auto& solid : m_solids)
        {
            for (auto& container : m_containers)
            {
                PushContact(solid, container, ContainInContainer(solid, container));
            }
        }
    }

    static CollisionManifold IntersectSolids(const AnyShape& a, const AnyShape& b)
    {
        switch (a.kind)
        {
        case ShapeKind::Sphere:
            switch (b.kind)
            {
            case ShapeKind::Sphere:   return Intersect(a.sphere, b.sphere);
            case ShapeKind::Plane:    return ForceNormalAtoB(Intersect(a.sphere, b.plane), a.sphere.center, b.plane.point);
            case ShapeKind::OBB:      return Intersect(a.sphere, b.obb);
            case ShapeKind::Capsule:  return Intersect(a.sphere, b.capsule);
            case ShapeKind::Cylinder: return Intersect(a.sphere, b.cylinder);
            }
            break;

        case ShapeKind::Plane:
            switch (b.kind)
            {
            case ShapeKind::Sphere:
            {
                auto m = FlipNormal(Intersect(b.sphere, a.plane));
                return ForceNormalAtoB(m, a.plane.point, b.sphere.center);
            }
            case ShapeKind::OBB:
            {
                auto m = FlipNormal(Intersect(b.obb, a.plane));
                return ForceNormalAtoB(m, a.plane.point, b.obb.center);
            }
            case ShapeKind::Capsule:
            {
                auto m = FlipNormal(Intersect(b.capsule, a.plane));
                return ForceNormalAtoB(m, a.plane.point, CapsuleCenter(b.capsule));
            }
            case ShapeKind::Cylinder:
            {
                auto m = FlipNormal(Intersect(b.cylinder, a.plane));
                return ForceNormalAtoB(m, a.plane.point, CylinderCenter(b.cylinder));
            }
            default: break;
            }
            break;

        case ShapeKind::OBB:
            switch (b.kind)
            {
            case ShapeKind::Sphere:   return FlipNormal(Intersect(b.sphere, a.obb));
            case ShapeKind::Plane:    return ForceNormalAtoB(Intersect(a.obb, b.plane), a.obb.center, b.plane.point);
            case ShapeKind::OBB:      return Intersect(a.obb, b.obb);
            case ShapeKind::Capsule:  return FlipNormal(Intersect(b.capsule, a.obb));
            case ShapeKind::Cylinder: return FlipNormal(Intersect(b.cylinder, a.obb));
            }
            break;

        case ShapeKind::Capsule:
            switch (b.kind)
            {
            case ShapeKind::Sphere:   return FlipNormal(Intersect(b.sphere, a.capsule));
            case ShapeKind::Plane:    return ForceNormalAtoB(Intersect(a.capsule, b.plane), CapsuleCenter(a.capsule), b.plane.point);
            case ShapeKind::OBB:      return Intersect(a.capsule, b.obb);
            case ShapeKind::Capsule:  return Intersect(a.capsule, b.capsule);
            case ShapeKind::Cylinder: return Intersect(a.capsule, b.cylinder);
            }
            break;

        case ShapeKind::Cylinder:
            switch (b.kind)
            {
            case ShapeKind::Sphere:   return FlipNormal(Intersect(b.sphere, a.cylinder));
            case ShapeKind::Plane:    return ForceNormalAtoB(Intersect(a.cylinder, b.plane), CylinderCenter(a.cylinder), b.plane.point);
            case ShapeKind::OBB:      return Intersect(a.cylinder, b.obb);
            case ShapeKind::Capsule:  return FlipNormal(Intersect(b.capsule, a.cylinder));
            case ShapeKind::Cylinder: return Intersect(a.cylinder, b.cylinder);
            }
            break;
        }
        return {};
    }

    static CollisionManifold ContainInContainer(const AnyShape& inner, const AnyShape& outer)
    {
        // NOTE: containment manifolds have their own conventions; leave as-is for now.
        // If you later see "container pushes outward", we can apply the same A->B forcing idea here.
        switch (outer.kind)
        {
        case ShapeKind::Sphere:
            switch (inner.kind)
            {
            case ShapeKind::Sphere:   return Contain(inner.sphere, outer.sphere);
            case ShapeKind::Capsule:  return Contain(inner.capsule, outer.sphere);
            case ShapeKind::Cylinder: return Contain(inner.cylinder, outer.sphere);
            case ShapeKind::OBB:      return Contain(inner.obb, outer.sphere);
            default: break;
            }
            break;

        case ShapeKind::OBB:
            switch (inner.kind)
            {
            case ShapeKind::Sphere:   return Contain(inner.sphere, outer.obb);
            case ShapeKind::Capsule:  return Contain(inner.capsule, outer.obb);
            case ShapeKind::Cylinder: return Contain(inner.cylinder, outer.obb);
            case ShapeKind::OBB:      return Contain(inner.obb, outer.obb);
            default: break;
            }
            break;

        case ShapeKind::Cylinder:
            switch (inner.kind)
            {
            case ShapeKind::Sphere:   return Contain(inner.sphere, outer.cylinder);
            case ShapeKind::Capsule:  return Contain(inner.capsule, outer.cylinder);
            case ShapeKind::Cylinder: return Contain(inner.cylinder, outer.cylinder);
            case ShapeKind::OBB:      return Contain(inner.obb, outer.cylinder);
            default: break;
            }
            break;

        case ShapeKind::Plane:
            switch (inner.kind)
            {
            case ShapeKind::Sphere:   return Contain(inner.sphere, outer.plane);
            case ShapeKind::Capsule:  return Contain(inner.capsule, outer.plane);
            case ShapeKind::Cylinder: return Contain(inner.cylinder, outer.plane);
            case ShapeKind::OBB:      return Contain(inner.obb, outer.plane);
            default: break;
            }
            break;

        default: break;
        }
        return {};
    }
};
