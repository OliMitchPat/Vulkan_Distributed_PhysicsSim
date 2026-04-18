#pragma once
#include "World.h"
#include "Components.h"

// From your physics library:
#include "Sphere.h"
#include "Plane.h"

#include <glm/glm.hpp>
#include <cmath>
#include <algorithm>

class CollisionSystem
{
public:
    void Update(World& world)
    {
        SphereVsPlane(world);
        SphereVsSphere(world);
    }

private:
    static float SphereWorldRadius(const TransformComponent& tr, const SphereColliderComponent& sc)
    {
        const float s = std::max(tr.scale.x, std::max(tr.scale.y, tr.scale.z));
        return sc.baseRadius * s;
    }

    static glm::vec3 SphereWorldCenter(const TransformComponent& tr, const SphereColliderComponent& sc)
    {
        return tr.position + sc.localCenter;
    }
    static void SphereVsPlane(World& world)
    {
        world.forEach<TransformComponent, PhysicsComponent>([&](Entity e, TransformComponent& tr, PhysicsComponent& phys)
            {
                auto* sc = world.getComponent<SphereColliderComponent>(e);
                if (!sc) return;

                const float r = SphereWorldRadius(tr, *sc);
                const glm::vec3 c = SphereWorldCenter(tr, *sc);

                Sphere sphereQuery(c, r);

                world.forEach<PlaneColliderComponent>([&](Entity planeE, PlaneColliderComponent& pc)
                    {
                        auto* planeTr = world.getComponent<TransformComponent>(planeE);
                        if (!planeTr) return;

                        Plane planeQuery(planeTr->position, pc.normal);

                        if (planeQuery.Intersects(sphereQuery))
                        {
                            const glm::vec3 n = planeQuery.GetNormal();      // normalized (declare ONCE)
                            const glm::vec3 v = phys.body.Velocity();
                            const glm::vec3 p0 = planeQuery.GetPosition();

                            // Only collide if moving INTO the plane
                            float vDotN = glm::dot(v, n);
                            if (vDotN < 0.0f)  // moving towards plane
                            {
                                // Coefficient of restitution (0 = no bounce, 1 = perfect bounce)
                                float restitution = 0.8f;  // or make configurable

                                // Impulse = -(1 + e) * (v · n) * n
                                glm::vec3 impulse = -(1.0f + restitution) * vDotN * n;

                                phys.body.ApplyImpulse(impulse);
                            }

                            // Depenetrate in a consistent direction
                            // Signed distance from center to plane
                            const float signedD = glm::dot(c - p0, n);

                            // We want the center to end up at distance r from the plane
                            // If signedD is positive, center is on normal side; push along +n
                            // If signedD is negative, center is on opposite side; push along -n
                            const float penetration = r - std::abs(signedD);

                            if (penetration > 0.0f)
                            {
                                const glm::vec3 pushDir = (signedD >= 0.0f) ? n : -n;

                                // Move transform (and physics) by penetration along the correct side
                                tr.position += pushDir * penetration;
                                phys.body.SetPosition(tr.position);
                            }
                        }
                    });
            });
    }

    static void SphereVsSphere(World& world)
    {
        world.forEach<TransformComponent, PhysicsComponent>([&](Entity aE, TransformComponent& aTr, PhysicsComponent& aPhys)
            {
                auto* aCol = world.getComponent<SphereColliderComponent>(aE);
                if (!aCol) return;

                const float ra = SphereWorldRadius(aTr, *aCol);
                const glm::vec3 ca = SphereWorldCenter(aTr, *aCol);
                Sphere aQuery(ca, ra);

                world.forEach<TransformComponent, PhysicsComponent>([&](Entity bE, TransformComponent& bTr, PhysicsComponent& bPhys)
                    {
                        if (bE <= aE) return;

                        auto* bCol = world.getComponent<SphereColliderComponent>(bE);
                        if (!bCol) return;

                        const float rb = SphereWorldRadius(bTr, *bCol);
                        const glm::vec3 cb = SphereWorldCenter(bTr, *bCol);
                        Sphere bQuery(cb, rb);

                        if (aQuery.CollideWith(bQuery))
                        {
                            const glm::vec3 delta = cb - ca;
                            const float distSq = glm::dot(delta, delta);
                            const float rSum = ra + rb;

                            if (distSq > 1e-8f)
                            {
                                const float dist = std::sqrt(distSq);
                                const glm::vec3 n = delta / dist;  // collision normal (from A to B)

                                // Get velocities
                                const glm::vec3 vA = aPhys.body.Velocity();
                                const glm::vec3 vB = bPhys.body.Velocity();

                                // Relative velocity (A relative to B)
                                const glm::vec3 vRel = vA - vB;
                                float vRelDotN = glm::dot(vRel, n);

                                // Only apply impulse if spheres are approaching
                                if (vRelDotN > 0.0f)  // A moving towards B
                                {
                                    float restitution = 0.8f;
                                    float mA = aPhys.body.Mass();
                                    float mB = bPhys.body.Mass();

                                    // Impulse calculation for two bodies
                                    // For equal mass: impulse magnitude = -(1+e) * vRel·n / 2
                                    float impulseMag = -(1.0f + restitution) * vRelDotN / (1.0f / mA + 1.0f / mB);

                                    glm::vec3 impulse = impulseMag * n;

                                    // Apply equal and opposite impulses
                                    aPhys.body.ApplyImpulse(impulse);
                                    bPhys.body.ApplyImpulse(-impulse);
                                }

                                // Depenetration: separate spheres if overlapping
                                const float penetration = rSum - dist;

                                if (penetration > 0.0f)
                                {
                                    const bool aMovable = (aPhys.body.InverseMass() > 0.0f);
                                    const bool bMovable = (bPhys.body.InverseMass() > 0.0f);

                                    if (aMovable && bMovable)
                                    {
                                        aTr.position -= n * (penetration * 0.5f);
                                        bTr.position += n * (penetration * 0.5f);
                                        aPhys.body.SetPosition(aTr.position);
                                        bPhys.body.SetPosition(bTr.position);
                                    }
                                    else if (aMovable && !bMovable)
                                    {
                                        aTr.position -= n * penetration;
                                        aPhys.body.SetPosition(aTr.position);
                                    }
                                    else if (!aMovable && bMovable)
                                    {
                                        bTr.position += n * penetration;
                                        bPhys.body.SetPosition(bTr.position);
                                    }
                                }
                            }
                            else
                            {
                                // Centers are basically identical; choose an arbitrary separation axis
                                // (prevents NaNs from normalization)
                                const glm::vec3 n(1.0f, 0.0f, 0.0f);
                                const float penetration = rSum;

                                const bool aMovable = (aPhys.body.InverseMass() > 0.0f);
                                const bool bMovable = (bPhys.body.InverseMass() > 0.0f);

                                if (aMovable && bMovable)
                                {
                                    aTr.position -= n * (penetration * 0.5f);
                                    bTr.position += n * (penetration * 0.5f);
                                    aPhys.body.SetPosition(aTr.position);
                                    bPhys.body.SetPosition(bTr.position);
                                }
                                else if (aMovable && !bMovable)
                                {
                                    aTr.position -= n * penetration;
                                    aPhys.body.SetPosition(aTr.position);
                                }
                                else if (!aMovable && bMovable)
                                {
                                    bTr.position += n * penetration;
                                    bPhys.body.SetPosition(bTr.position);
                                }
                            }
                        }
                    });
            });
    }
};