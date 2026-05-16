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
#include <limits>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>

class CollisionSystem
{
public:
    void Update(World& world)
    {
        m_contacts.clear();
        m_solids.clear();
        m_containers.clear();
        m_spatialGrid.clear();
        m_seenPairs.clear();
        m_lastSolidCount = 0;
        m_lastContainerCount = 0;
        m_lastCandidatePairs = 0;
        m_lastBroadPhaseRejectedPairs = 0;
        m_lastContacts = 0;
        m_lastSpatialCells = 0;
        m_lastSolverVelocityIterations = 0;
        m_lastSolidSolidCandidatePairs = 0;
        m_lastSolidContainerCandidatePairs = 0;
        m_lastContainerBroadPhaseRejectedPairs = 0;

        GatherShapes(world);
        m_lastSolidCount = static_cast<uint32_t>(m_solids.size());
        m_lastContainerCount = static_cast<uint32_t>(m_containers.size());

        TestSolidSolid();
        TestSolidContainer();
        m_lastContacts = static_cast<uint32_t>(m_contacts.size());

        const int velocityIterations =
            m_contacts.size() > 512 ? 2 :
            m_contacts.size() > 256 ? 4 :
            8;
        m_lastSolverVelocityIterations = static_cast<uint32_t>(velocityIterations);

        // First solve velocity impulses multiple times.
        // This handles bounce, stopping, friction, and angular response.
        for (int i = 0; i < velocityIterations; ++i)
        {
            for (auto& c : m_contacts)
            {
                ResolveContactVelocity(*c.A, *c.B, c.manifold, c.material);
            }
        }

        const int positionIterations = 1;

        // Apply positional correction a small number of times.
        // This is strong enough to prevent sinking, but much less jittery
        // than doing it inside the full velocity solver loop.
        for (int i = 0; i < positionIterations; ++i)
        {
            for (auto& c : m_contacts)
            {
                ResolveContactPosition(*c.A, *c.B, c.manifold);
            }
        }

        // Milestone 5: only authoritative dynamic bodies should drive ECS transforms here.
        // Replicas are kinematic and should be driven by received snapshots (not by collision).
        world.forEach<TransformComponent, PhysicsComponent>(
            [&](Entity, TransformComponent& tr, PhysicsComponent& phys)
            {
                if (!phys.body.IsDynamic())
                    return;

                tr.position = phys.body.Position();
                // (Rotation sync is done in PhysicsSystem.Update; keep minimal here.)
            });
    }

    uint32_t LastSolidCount() const { return m_lastSolidCount; }
    uint32_t LastContainerCount() const { return m_lastContainerCount; }
    uint32_t LastCandidatePairs() const { return m_lastCandidatePairs; }
    uint32_t LastBroadPhaseRejectedPairs() const { return m_lastBroadPhaseRejectedPairs; }
    uint32_t LastContactCount() const { return m_lastContacts; }
    uint32_t LastSpatialCells() const { return m_lastSpatialCells; }
    uint32_t LastSolverVelocityIterations() const { return m_lastSolverVelocityIterations; }
    uint32_t LastSolidSolidCandidatePairs() const { return m_lastSolidSolidCandidatePairs; }
    uint32_t LastSolidContainerCandidatePairs() const { return m_lastSolidContainerCandidatePairs; }
    uint32_t LastContainerBroadPhaseRejectedPairs() const { return m_lastContainerBroadPhaseRejectedPairs; }

private:
    struct BodyRef
    {
        RigidBody* body = nullptr;
        float restitution = 0.5f;
        float staticFriction = 0.5f;
        float dynamicFriction = 0.3f;
        bool dynamic = false;
    };

    enum class ShapeKind { Sphere, Plane, Capsule, Cylinder, OBB };

    struct AnyShape
    {
        ShapeKind kind;
        BodyRef   br;
        glm::vec3 boundCenter{ 0.0f };
        float boundRadius = 0.0f;
        bool hasFiniteBound = false;
        glm::vec3 aabbMin{ 0.0f };
        glm::vec3 aabbMax{ 0.0f };
        bool hasAabb = false;
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
            AnyShape s; s.kind = ShapeKind::Sphere; s.br = br; s.sphere = ws;
            s.boundCenter = ws.center; s.boundRadius = ws.radius; s.hasFiniteBound = true;
            s.aabbMin = ws.center - glm::vec3(ws.radius);
            s.aabbMax = ws.center + glm::vec3(ws.radius);
            s.hasAabb = true;
            return s;
        }

        static AnyShape MakePlane(const BodyRef& br, const WorldPlane& wp)
        {
            AnyShape s; s.kind = ShapeKind::Plane; s.br = br; s.plane = wp;
            s.hasFiniteBound = false;
            return s;
        }

        static AnyShape MakeCapsule(const BodyRef& br, const WorldCapsule& wc)
        {
            AnyShape s; s.kind = ShapeKind::Capsule; s.br = br; s.capsule = wc;
            s.boundCenter = 0.5f * (wc.a + wc.b);
            s.boundRadius = 0.5f * glm::length(wc.b - wc.a) + wc.radius;
            s.hasFiniteBound = true;
            s.aabbMin = glm::min(wc.a, wc.b) - glm::vec3(wc.radius);
            s.aabbMax = glm::max(wc.a, wc.b) + glm::vec3(wc.radius);
            s.hasAabb = true;
            return s;
        }

        static AnyShape MakeCylinder(const BodyRef& br, const WorldCylinder& wy)
        {
            AnyShape s; s.kind = ShapeKind::Cylinder; s.br = br; s.cylinder = wy;
            s.boundCenter = 0.5f * (wy.a + wy.b);
            s.boundRadius = 0.5f * glm::length(wy.b - wy.a) + wy.radius;
            s.hasFiniteBound = true;
            s.aabbMin = glm::min(wy.a, wy.b) - glm::vec3(wy.radius);
            s.aabbMax = glm::max(wy.a, wy.b) + glm::vec3(wy.radius);
            s.hasAabb = true;
            return s;
        }

        static AnyShape MakeOBB(const BodyRef& br, const WorldOBB& wo)
        {
            AnyShape s; s.kind = ShapeKind::OBB; s.br = br; s.obb = wo;
            s.boundCenter = wo.center;
            s.boundRadius = glm::length(wo.halfExtents);
            s.hasFiniteBound = true;
            const glm::vec3 extents =
                glm::abs(wo.axisX) * wo.halfExtents.x +
                glm::abs(wo.axisY) * wo.halfExtents.y +
                glm::abs(wo.axisZ) * wo.halfExtents.z;
            s.aabbMin = wo.center - extents;
            s.aabbMax = wo.center + extents;
            s.hasAabb = true;
            return s;
        }
    };

    struct GridKey
    {
        int x = 0;
        int y = 0;
        int z = 0;

        bool operator==(const GridKey& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct GridKeyHash
    {
        size_t operator()(const GridKey& key) const
        {
            size_t h = static_cast<size_t>(key.x) * 73856093u;
            h ^= static_cast<size_t>(key.y) * 19349663u;
            h ^= static_cast<size_t>(key.z) * 83492791u;
            return h;
        }
    };

    std::vector<AnyShape> m_solids;
    std::vector<AnyShape> m_containers;
    std::unordered_map<GridKey, std::vector<size_t>, GridKeyHash> m_spatialGrid;
    std::unordered_set<uint64_t> m_seenPairs;
    uint32_t m_lastSolidCount = 0;
    uint32_t m_lastContainerCount = 0;
    uint32_t m_lastCandidatePairs = 0;
    uint32_t m_lastBroadPhaseRejectedPairs = 0;
    uint32_t m_lastContacts = 0;
    uint32_t m_lastSpatialCells = 0;
    uint32_t m_lastSolverVelocityIterations = 0;
    uint32_t m_lastSolidSolidCandidatePairs = 0;
    uint32_t m_lastSolidContainerCandidatePairs = 0;
    uint32_t m_lastContainerBroadPhaseRejectedPairs = 0;

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

        m_contacts.push_back({ a.br.body, b.br.body, m, CombineMaterial(a.br, b.br) });
    }

    static BodyRef MakeBodyRef(PhysicsComponent& phys)
    {
        BodyRef br{};
        br.body = &phys.body;
        br.restitution = phys.restitution;
        br.staticFriction = phys.staticFriction;
        br.dynamicFriction = phys.dynamicFriction;
        br.dynamic = phys.body.IsDynamic();
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

    static bool BoundingSphere(const AnyShape& s, glm::vec3& center, float& radius)
    {
        if (!s.hasFiniteBound)
            return false;

        center = s.boundCenter;
        radius = s.boundRadius;
        return true;
    }

    static bool AabbReject(const AnyShape& a, const AnyShape& b)
    {
        if (!a.hasAabb || !b.hasAabb)
            return false;

        return
            a.aabbMax.x < b.aabbMin.x || a.aabbMin.x > b.aabbMax.x ||
            a.aabbMax.y < b.aabbMin.y || a.aabbMin.y > b.aabbMax.y ||
            a.aabbMax.z < b.aabbMin.z || a.aabbMin.z > b.aabbMax.z;
    }

    static bool BroadPhaseReject(const AnyShape& a, const AnyShape& b)
    {
        if (AabbReject(a, b))
            return true;

        glm::vec3 centerA{ 0.0f };
        glm::vec3 centerB{ 0.0f };
        float radiusA = 0.0f;
        float radiusB = 0.0f;
        if (!BoundingSphere(a, centerA, radiusA) || !BoundingSphere(b, centerB, radiusB))
            return false;

        const float radiusSum = radiusA + radiusB;
        const glm::vec3 delta = centerB - centerA;
        return glm::dot(delta, delta) > radiusSum * radiusSum;
    }

    static uint64_t PairKey(size_t a, size_t b)
    {
        if (a > b)
            std::swap(a, b);
        return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
            static_cast<uint32_t>(b);
    }

    static int CellCoord(float value, float cellSize)
    {
        return static_cast<int>(std::floor(value / cellSize));
    }

    float EstimateSpatialCellSize() const
    {
        float totalRadius = 0.0f;
        uint32_t boundedCount = 0;
        for (const auto& solid : m_solids)
        {
            glm::vec3 center{ 0.0f };
            float radius = 0.0f;
            if (!BoundingSphere(solid, center, radius))
                continue;

            totalRadius += radius;
            ++boundedCount;
        }

        if (boundedCount == 0)
            return 4.0f;

        const float avgRadius = totalRadius / static_cast<float>(boundedCount);
        return glm::clamp(avgRadius * 4.0f, 2.0f, 12.0f);
    }

    void TestSolidPair(size_t i, size_t j)
    {
        if (i == j)
            return;

        const AnyShape& a = m_solids[i];
        const AnyShape& b = m_solids[j];
        if (!a.br.dynamic && !b.br.dynamic)
            return;

        if (BroadPhaseReject(a, b))
        {
            ++m_lastBroadPhaseRejectedPairs;
            return;
        }

        ++m_lastCandidatePairs;
        ++m_lastSolidSolidCandidatePairs;
        PushContact(a, b, IntersectSolids(a, b));
    }

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
        if (m_solids.size() >= 64)
        {
            TestSolidSolidSpatial();
            return;
        }

        for (std::size_t i = 0; i < m_solids.size(); ++i)
        {
            for (std::size_t j = i + 1; j < m_solids.size(); ++j)
                TestSolidPair(i, j);
        }
    }

    void TestSolidSolidSpatial()
    {
        m_spatialGrid.clear();
        m_seenPairs.clear();
        m_seenPairs.reserve(m_solids.size() * 8);

        std::vector<size_t> globalSolids;
        const float cellSize = EstimateSpatialCellSize();

        for (size_t i = 0; i < m_solids.size(); ++i)
        {
            glm::vec3 center{ 0.0f };
            float radius = 0.0f;
            if (!BoundingSphere(m_solids[i], center, radius))
            {
                globalSolids.push_back(i);
                continue;
            }

            const glm::vec3 minBound = m_solids[i].hasAabb ? m_solids[i].aabbMin : center - glm::vec3(radius);
            const glm::vec3 maxBound = m_solids[i].hasAabb ? m_solids[i].aabbMax : center + glm::vec3(radius);

            const int minX = CellCoord(minBound.x, cellSize);
            const int maxX = CellCoord(maxBound.x, cellSize);
            const int minY = CellCoord(minBound.y, cellSize);
            const int maxY = CellCoord(maxBound.y, cellSize);
            const int minZ = CellCoord(minBound.z, cellSize);
            const int maxZ = CellCoord(maxBound.z, cellSize);
            const int cellSpan =
                (maxX - minX + 1) *
                (maxY - minY + 1) *
                (maxZ - minZ + 1);

            if (cellSpan > 512)
            {
                globalSolids.push_back(i);
                continue;
            }

            for (int z = minZ; z <= maxZ; ++z)
            {
                for (int y = minY; y <= maxY; ++y)
                {
                    for (int x = minX; x <= maxX; ++x)
                        m_spatialGrid[GridKey{ x, y, z }].push_back(i);
                }
            }
        }

        m_lastSpatialCells = static_cast<uint32_t>(m_spatialGrid.size());

        for (const auto& bucket : m_spatialGrid)
        {
            const auto& indices = bucket.second;
            for (size_t a = 0; a < indices.size(); ++a)
            {
                for (size_t b = a + 1; b < indices.size(); ++b)
                {
                    const size_t i = indices[a];
                    const size_t j = indices[b];
                    const uint64_t key = PairKey(i, j);
                    if (!m_seenPairs.insert(key).second)
                        continue;

                    TestSolidPair(i, j);
                }
            }
        }

        for (const size_t globalIndex : globalSolids)
        {
            for (size_t other = 0; other < m_solids.size(); ++other)
            {
                if (other == globalIndex)
                    continue;

                const uint64_t key = PairKey(globalIndex, other);
                if (!m_seenPairs.insert(key).second)
                    continue;

                TestSolidPair(globalIndex, other);
            }
        }
    }

    void TestSolidContainer()
    {
        for (auto& solid : m_solids)
        {
            if (!solid.br.dynamic)
                continue;

            for (auto& container : m_containers)
            {
                if (BroadPhaseReject(solid, container))
                {
                    ++m_lastBroadPhaseRejectedPairs;
                    ++m_lastContainerBroadPhaseRejectedPairs;
                    continue;
                }

                ++m_lastCandidatePairs;
                ++m_lastSolidContainerCandidatePairs;
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
        CollisionManifold m{};

        switch (outer.kind)
        {
        case ShapeKind::Sphere:
            switch (inner.kind)
            {
            case ShapeKind::Sphere:   m = Contain(inner.sphere, outer.sphere); break;
            case ShapeKind::Capsule:  m = Contain(inner.capsule, outer.sphere); break;
            case ShapeKind::Cylinder: m = Contain(inner.cylinder, outer.sphere); break;
            case ShapeKind::OBB:      m = Contain(inner.obb, outer.sphere); break;
            default: break;
            }
            break;

        case ShapeKind::OBB:
            switch (inner.kind)
            {
            case ShapeKind::Sphere:   m = Contain(inner.sphere, outer.obb); break;
            case ShapeKind::Capsule:  m = Contain(inner.capsule, outer.obb); break;
            case ShapeKind::Cylinder: m = Contain(inner.cylinder, outer.obb); break;
            case ShapeKind::OBB:      m = Contain(inner.obb, outer.obb); break;
            default: break;
            }
            break;

        case ShapeKind::Cylinder:
            switch (inner.kind)
            {
            case ShapeKind::Sphere:   m = Contain(inner.sphere, outer.cylinder); break;
            case ShapeKind::Capsule:  m = Contain(inner.capsule, outer.cylinder); break;
            case ShapeKind::Cylinder: m = Contain(inner.cylinder, outer.cylinder); break;
            case ShapeKind::OBB:      m = Contain(inner.obb, outer.cylinder); break;
            default: break;
            }
            break;

        case ShapeKind::Capsule:
            switch (inner.kind)
            {
            case ShapeKind::Sphere:   m = Contain(inner.sphere, outer.capsule); break;
            case ShapeKind::Capsule:  m = Contain(inner.capsule, outer.capsule); break;
            case ShapeKind::Cylinder: m = Contain(inner.cylinder, outer.capsule); break;
            case ShapeKind::OBB:      m = Contain(inner.obb, outer.capsule); break;
            default: break;
            }
            break;

        case ShapeKind::Plane:
            switch (inner.kind)
            {
            case ShapeKind::Sphere:   m = Contain(inner.sphere, outer.plane); break;
            case ShapeKind::Capsule:  m = Contain(inner.capsule, outer.plane); break;
            case ShapeKind::Cylinder: m = Contain(inner.cylinder, outer.plane); break;
            case ShapeKind::OBB:      m = Contain(inner.obb, outer.plane); break;
            default: break;
            }
            break;

        default: break;
        }

        if (m.hit)
        {
            // Containment routines return an inward correction direction for the inner body.
            // ResolveContact expects manifold normal in A->B order (A=inner solid, B=container),
            // so flip once here before pushing solver contacts.
            m.normal = -m.normal;
        }

        return m;
    }
};
