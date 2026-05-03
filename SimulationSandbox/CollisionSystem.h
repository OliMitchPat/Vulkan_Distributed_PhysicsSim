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
#include "ShapeData.h"

#include <glm/glm.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

class CollisionSystem
{
public:
    void Update(World& world)
    {
        m_contacts.clear();

        BuildSpherePlaneContacts(world);
        BuildSphereSphereContacts(world);

        // Solve contacts multiple times (important for stability)
        const int iterations = 8;
        for (int i = 0; i < iterations; i++)
        {
            for (auto& c : m_contacts)
            {
                SolveContactImpulse(*c.A, *c.B, c.manifold, c.material);
            }
        }

        // Positional correction (after impulses)
        for (auto& c : m_contacts)
        {
            PositionalCorrection(*c.A, *c.B, c.manifold);
        }
    }

private:
    struct Contact
    {
        RigidBody* A;
        RigidBody* B;
        CollisionManifold manifold;
        ContactMaterial material;
    };

    std::vector<Contact> m_contacts;

    // =========================================================

    static float SphereWorldRadius(const TransformComponent& tr, const SphereColliderComponent& sc)
    {
        const float s = std::max({ tr.scale.x, tr.scale.y, tr.scale.z });
        return sc.baseRadius * s;
    }

    static glm::vec3 SphereWorldCenter(const TransformComponent& tr, const SphereColliderComponent& sc)
    {
        return tr.position + sc.localCenter;
    }

    // =========================================================

  // --- inside CollisionSystem.h ---

    void BuildSpherePlaneContacts(World& world)
    {
        world.forEach<TransformComponent, PhysicsComponent>(
            [&](Entity e, TransformComponent& tr, PhysicsComponent& phys)
            {
                auto* sc = world.getComponent<SphereColliderComponent>(e);
                if (!sc) return;

                // Build world-sphere from your ECS data
                WorldSphere ws{};
                ws.center = SphereWorldCenter(tr, *sc);
                ws.radius = SphereWorldRadius(tr, *sc);

                // Iterate planes
                world.forEach<TransformComponent, PlaneColliderComponent>(
                    [&](Entity, TransformComponent& planeTr, PlaneColliderComponent& pc)
                    {
                        // Build world-plane from ECS data
                        WorldPlane wp{};
                        wp.point = planeTr.position;
                        wp.normal = glm::normalize(pc.normal);

                        // Narrow-phase intersection is now a free function
                        CollisionManifold m = Intersect(ws, wp);
                        if (!m.hit) return;

                        ContactMaterial mat{};
                        mat.restitution = 0.8f;

                        // Static plane body
                        static RigidBody staticPlane;
                        staticPlane.SetMotionType(BodyMotionType::Static);

                        m_contacts.push_back({
                            &phys.body,
                            &staticPlane,
                            m,
                            mat
                            });
                    });
            });
    }

    void BuildSphereSphereContacts(World& world)
    {
        world.forEach<TransformComponent, PhysicsComponent>(
            [&](Entity aE, TransformComponent& aTr, PhysicsComponent& aPhys)
            {
                auto* aCol = world.getComponent<SphereColliderComponent>(aE);
                if (!aCol) return;

                WorldSphere a{};
                a.center = SphereWorldCenter(aTr, *aCol);
                a.radius = SphereWorldRadius(aTr, *aCol);

                world.forEach<TransformComponent, PhysicsComponent>(
                    [&](Entity bE, TransformComponent& bTr, PhysicsComponent& bPhys)
                    {
                        if (bE <= aE) return;

                        auto* bCol = world.getComponent<SphereColliderComponent>(bE);
                        if (!bCol) return;

                        WorldSphere b{};
                        b.center = SphereWorldCenter(bTr, *bCol);
                        b.radius = SphereWorldRadius(bTr, *bCol);

                        CollisionManifold m = Intersect(a, b);
                        if (!m.hit) return;

                        ContactMaterial mat{};
                        mat.restitution = 0.8f;

                        m_contacts.push_back({
                            &aPhys.body,
                            &bPhys.body,
                            m,
                            mat
                            });
                    });
            });
    }

    // =========================================================

    static void PositionalCorrection(RigidBody& A, RigidBody& B, const CollisionManifold& m)
    {
        const float percent = 0.8f;   // correction strength
        const float slop = 0.01f;     // penetration allowance

        float invMassA = A.EffectiveInverseMass();
        float invMassB = B.EffectiveInverseMass();

        float totalInvMass = invMassA + invMassB;
        if (totalInvMass <= 0.0f) return;

        float correctionMag = std::max(m.penetration - slop, 0.0f) / totalInvMass;
        glm::vec3 correction = percent * correctionMag * m.normal;

        if (A.IsDynamic())
            A.SetPosition(A.Position() - correction * invMassA);

        if (B.IsDynamic())
            B.SetPosition(B.Position() + correction * invMassB);
    }
};