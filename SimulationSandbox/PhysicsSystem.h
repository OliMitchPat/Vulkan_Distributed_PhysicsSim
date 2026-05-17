#pragma once

#include <glm/glm.hpp>

#include "World.h"
#include "Components.h"
#include "RigidBody.h"
#include "BodySetup.h"
#include "ShapeData.h"
#include "ThreadUtils.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

class PhysicsSystem
{
    struct DynamicBodyEntry
    {
        Entity entity = INVALID_ENTITY;
        PhysicsComponent* physics = nullptr;
        TransformComponent* transform = nullptr;
        VelocityComponent* velocity = nullptr;
    };

public:
    ~PhysicsSystem()
    {
        StopWorkers();
    }

    void InitializePhysicsBody(World& world, Entity e)
    {
        auto* phys = world.getComponent<PhysicsComponent>(e);
        auto* tr = world.getComponent<TransformComponent>(e);
        if (!phys || !tr || phys->initialized)
            return;

        phys->body.SetPosition(tr->position);

        glm::quat q = glm::quat(tr->rotation);
        phys->body.SetOrientation(q);

        bool isContainer = false;
        if (auto* shape = world.getComponent<ShapeComponent>(e))
        {
            isContainer = (shape->collisionType == CollisionType::CONTAINER);

            std::visit([&](auto&& s)
                {
                    using T = std::decay_t<decltype(s)>;

                    if constexpr (std::is_same_v<T, SphereShape>)
                        SetupSphereBody(phys->body, phys->density, s.radius);
                    else if constexpr (std::is_same_v<T, CuboidShape>)
                        SetupCuboidBody(phys->body, phys->density, s.size);
                    else if constexpr (std::is_same_v<T, CylinderShape>)
                        SetupCylinderBody(phys->body, phys->density, s.radius, s.height);
                    else if constexpr (std::is_same_v<T, CapsuleShape>)
                        SetupCapsuleBody(phys->body, phys->density, s.radius, s.height);
                    else if constexpr (std::is_same_v<T, PlaneShape>)
                        SetStaticBody(phys->body);
                }, shape->shape);
        }

        if (phys->behaviour == BodyMotionType::Static)
        {
            phys->body.SetMotionType(BodyMotionType::Static);
        }
        else if (isContainer)
        {
            phys->body.SetMotionType(BodyMotionType::Static);
        }
        else if (world.getComponent<AnimatedPathComponent>(e))
        {
            phys->body.SetMotionType(BodyMotionType::Kinematic);
        }
        else if (auto* owner = world.getComponent<OwnerComponent>(e))
        {
            if (owner->ownerId >= 0)
            {
                const bool isOwned = (owner->ownerId == (m_localPeerId - 1));
                phys->body.SetMotionType(isOwned ? BodyMotionType::Dynamic : BodyMotionType::Kinematic);
            }
            else
            {
                phys->body.SetMotionType(BodyMotionType::Static);
            }
        }
        else
        {
            phys->body.SetMotionType(BodyMotionType::Static);
        }

        if (phys->body.IsDynamic())
        {
            phys->body.SetLinearDamping(0.03f);
            phys->body.SetAngularDamping(0.08f);
        }
        else
        {
            phys->body.SetLinearDamping(0.0f);
            phys->body.SetAngularDamping(0.0f);
        }

        if (auto* vel = world.getComponent<VelocityComponent>(e))
        {
            phys->body.SetLinearVelocity(vel->linearVelocity);
            phys->body.SetAngularVelocity(vel->angularVelocity);
        }

        phys->initialized = true;
    }

    void InitializePhysicsBodies(World& world)
    {
        world.forEach<PhysicsComponent, TransformComponent>(
            [&](Entity e, PhysicsComponent&, TransformComponent&)
            {
                InitializePhysicsBody(world, e);
            });
        RebuildDynamicEntityCache(world);
    }

    void RebuildDynamicEntityCache(World& world)
    {
        m_dynamicBodies.clear();

        world.forEach<PhysicsComponent>(
            [&](Entity e, PhysicsComponent& phys)
            {
                if (!phys.initialized || !phys.body.IsDynamic())
                    return;

                auto* owner = world.getComponent<OwnerComponent>(e);
                if (!owner || owner->ownerId != m_localPeerId - 1)
                    return;

                DynamicBodyEntry entry{};
                entry.entity = e;
                entry.physics = &phys;
                entry.transform = world.getComponent<TransformComponent>(e);
                entry.velocity = world.getComponent<VelocityComponent>(e);
                m_dynamicBodies.push_back(entry);
            });
    }

    size_t DynamicBodyCount() const { return m_dynamicBodies.size(); }

    void Update(World& world, float dt)
    {
        const glm::vec3 gravity(0.0f, -9.81f, 0.0f);

        const std::size_t workerCount =
            std::max<std::size_t>(1, std::min<std::size_t>(m_workerCount, m_dynamicBodies.size()));

        ParallelFor(m_dynamicBodies.size(), workerCount,
            [&](std::size_t begin, std::size_t end)
            {
                for (std::size_t i = begin; i < end; ++i)
                {
                    DynamicBodyEntry& entry = m_dynamicBodies[i];
                    PhysicsComponent* phys = entry.physics;

                    if (m_gravityEnabled)
                        phys->body.AddForce(gravity * phys->body.Mass());

                    phys->body.Integrate(dt, m_integrator);

                    if (entry.transform)
                    {
                        entry.transform->position = phys->body.Position();
                        entry.transform->orientation = phys->body.Orientation();
                        entry.transform->rotation = glm::eulerAngles(entry.transform->orientation);
                    }

                    if (entry.velocity)
                    {
                        entry.velocity->linearVelocity = phys->body.LinearVelocity();
                        entry.velocity->angularVelocity = phys->body.AngularVelocity();
                    }
                }
            });
    }

    // --------------------------------------------------
    // Config
    // --------------------------------------------------
    void SetIntegrator(IntegratorType type) { m_integrator = type; }
    IntegratorType GetIntegrator() const { return m_integrator; }

    void SetGravityEnabled(bool enabled) { m_gravityEnabled = enabled; }
    bool IsGravityEnabled() const { return m_gravityEnabled; }

    void SetWorkerCount(int workers)
    {
        m_workerCount = std::max(1, workers);
        EnsureWorkers();
    }
    int  GetWorkerCount() const { return m_workerCount; }
    void SetWorkerCores(const std::vector<int>& cores)
    {
        m_workerCores = cores;
        EnsureWorkers();
    }

    // Milestone 5: used to decide authoritative ownership
    void SetLocalPeerId(int peerId_1_to_4) { m_localPeerId = peerId_1_to_4; }
    int  GetLocalPeerId() const { return m_localPeerId; }

private:
    template<typename Fn>
    void ParallelFor(std::size_t count, std::size_t workerCount, Fn fn)
    {
        if (count == 0)
            return;

        if (workerCount <= 1 || count < 2)
        {
            fn(0, count);
            return;
        }

        EnsureWorkers();

        const std::size_t workerThreadCount =
            std::min<std::size_t>(m_workers.size(), workerCount - 1);

        if (workerThreadCount == 0 || count < 256)
        {
            fn(0, count);
            return;
        }

        const std::size_t lanes = workerThreadCount + 1;
        const std::size_t chunk = (count + lanes - 1) / lanes;

        {
            std::unique_lock<std::mutex> lk(m_workerMutex);
            m_ranges.clear();
            m_ranges.reserve(workerThreadCount);
            m_taskFn = [&](std::size_t begin, std::size_t end) { fn(begin, end); };

            std::size_t begin = 0;
            for (std::size_t w = 0; w < workerThreadCount && begin + chunk < count; ++w)
            {
                const std::size_t end = std::min(count, begin + chunk);
                m_ranges.push_back({ begin, end });
                begin = end;
            }

            m_mainRange = { begin, count };
            m_pendingWorkers = m_ranges.size();
            ++m_taskGeneration;
        }

        m_workerCv.notify_all();

        fn(m_mainRange.begin, m_mainRange.end);

        std::unique_lock<std::mutex> lk(m_workerMutex);
        m_doneCv.wait(lk, [&]() { return m_pendingWorkers == 0; });
    }

    struct WorkRange
    {
        std::size_t begin = 0;
        std::size_t end = 0;
    };

    void EnsureWorkers()
    {
        const std::size_t desired =
            (m_workerCount > 1) ? static_cast<std::size_t>(m_workerCount - 1) : 0;

        if (m_workers.size() == desired)
            return;

        StopWorkers();

        m_stopWorkers = false;
        for (std::size_t i = 0; i < desired; ++i)
        {
            m_workers.emplace_back([this, i]()
            {
                if (!m_workerCores.empty())
                {
                    const int core = m_workerCores[i % m_workerCores.size()];
                    ThreadUtils::PinCurrentThreadToCore(core);
                }
                ThreadUtils::SetCurrentThreadName("SimWorker");
                WorkerLoop(i);
            });
        }
    }

    void StopWorkers()
    {
        {
            std::lock_guard<std::mutex> lk(m_workerMutex);
            m_stopWorkers = true;
            ++m_taskGeneration;
        }
        m_workerCv.notify_all();

        for (auto& worker : m_workers)
        {
            if (worker.joinable())
                worker.join();
        }
        m_workers.clear();
        m_stopWorkers = false;
        m_pendingWorkers = 0;
    }

    void WorkerLoop(std::size_t workerIndex)
    {
        uint64_t seenGeneration = 0;

        for (;;)
        {
            WorkRange range{};
            std::function<void(std::size_t, std::size_t)> task;

            {
                std::unique_lock<std::mutex> lk(m_workerMutex);
                m_workerCv.wait(lk, [&]()
                {
                    return m_stopWorkers || m_taskGeneration != seenGeneration;
                });

                if (m_stopWorkers)
                    return;

                seenGeneration = m_taskGeneration;
                if (workerIndex >= m_ranges.size())
                    continue;

                range = m_ranges[workerIndex];
                task = m_taskFn;
            }

            if (task && range.begin < range.end)
                task(range.begin, range.end);

            {
                std::lock_guard<std::mutex> lk(m_workerMutex);
                if (m_pendingWorkers > 0)
                    --m_pendingWorkers;
            }
            m_doneCv.notify_one();
        }
    }

    IntegratorType m_integrator = IntegratorType::SemiImplicitEuler;
    bool m_gravityEnabled = true;
    int m_workerCount = 1;
    std::vector<int> m_workerCores;
    std::vector<DynamicBodyEntry> m_dynamicBodies;
    std::vector<std::thread> m_workers;
    std::mutex m_workerMutex;
    std::condition_variable m_workerCv;
    std::condition_variable m_doneCv;
    std::vector<WorkRange> m_ranges;
    WorkRange m_mainRange{};
    std::function<void(std::size_t, std::size_t)> m_taskFn;
    std::size_t m_pendingWorkers = 0;
    uint64_t m_taskGeneration = 0;
    bool m_stopWorkers = false;

    int m_localPeerId = 1; // config peer_id (1..4)
};
