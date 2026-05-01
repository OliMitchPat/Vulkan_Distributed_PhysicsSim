#pragma once
#include "World.h"
#include "Components.h"

#include "Sphere.h"
#include "Plane.h"

#include "CollisionManifold.h"
#include "ImpulseSolver.h"

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

    void BuildSpherePlaneContacts(World& world)
    {
        world.forEach<TransformComponent, PhysicsComponent>([&](Entity e, TransformComponent& tr, PhysicsComponent& phys)
            {
                auto* sc = world.getComponent<SphereColliderComponent>(e);
                if (!sc) return;

                const float r = SphereWorldRadius(tr, *sc);
                const glm::vec3 c = SphereWorldCenter(tr, *sc);

                Sphere sphere(c, r);

                world.forEach<TransformComponent, PlaneColliderComponent>([&](Entity planeE, TransformComponent& planeTr, PlaneColliderComponent& pc)
                    {
                        Plane plane(planeTr.position, pc.normal);

                        if (!plane.Intersects(sphere)) return;

                        const glm::vec3 n = plane.GetNormal();
                        const float signedD = glm::dot(c - planeTr.position, n);
                        const float penetration = r - std::abs(signedD);

                        if (penetration <= 0.0f) return;

                        CollisionManifold m;
                        m.hit = true;
                        m.normal = (signedD >= 0.0f) ? n : -n;
                        m.penetration = penetration;
                        m.contactPoint = c - m.normal * r;

                        ContactMaterial mat;
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

    // =========================================================

    void BuildSphereSphereContacts(World& world)
    {
        world.forEach<TransformComponent, PhysicsComponent>([&](Entity aE, TransformComponent& aTr, PhysicsComponent& aPhys)
            {
                auto* aCol = world.getComponent<SphereColliderComponent>(aE);
                if (!aCol) return;

                const float ra = SphereWorldRadius(aTr, *aCol);
                const glm::vec3 ca = SphereWorldCenter(aTr, *aCol);

                world.forEach<TransformComponent, PhysicsComponent>([&](Entity bE, TransformComponent& bTr, PhysicsComponent& bPhys)
                    {
                        if (bE <= aE) return;

                        auto* bCol = world.getComponent<SphereColliderComponent>(bE);
                        if (!bCol) return;

                        const float rb = SphereWorldRadius(bTr, *bCol);
                        const glm::vec3 cb = SphereWorldCenter(bTr, *bCol);

                        const glm::vec3 delta = cb - ca;
                        const float distSq = glm::dot(delta, delta);
                        const float rSum = ra + rb;

                        if (distSq >= rSum * rSum) return;

                        glm::vec3 n;
                        float dist;

                        if (distSq > 1e-8f)
                        {
                            dist = std::sqrt(distSq);
                            n = delta / dist;
                        }
                        else
                        {
                            // fallback axis
                            n = glm::vec3(1, 0, 0);
                            dist = rSum;
                        }

                        const float penetration = rSum - dist;

                        CollisionManifold m;
                        m.hit = true;
                        m.normal = n;
                        m.penetration = penetration;
                        m.contactPoint = ca + n * ra;

                        ContactMaterial mat;
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