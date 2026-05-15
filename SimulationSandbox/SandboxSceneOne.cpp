/*
 * SandboxSceneOne.cpp — Core Concurrency Threading Scaffold + Milestones 4/5
 *
 * Threads:
 *   Render thread  — GLFW, Vulkan, ImGui, camera control
 *   Sim thread     — ECS World, physics, collision, snapshot
 *   Net thread     — UDP send/recv (GLOBAL_COMMAND + STATE_SNAPSHOT)
 *
 * Rules:
 *   - Sim thread is the ONLY thread that mutates the live World.
 *   - Net thread only talks to sockets and writes to thread-safe shared buffers.
 *   - Render thread only consumes WorldSnapshot + controls UI.
 */
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <Windows.h>

#include "World.h"
#include "ScenarioManager.h"
#include "Scenario.h"
#include "Renderer.h"
#include "RenderScene.h"
#include "imgui.h"
#include <GLFW/glfw3.h>

#include "Scenario_PrimitiveScene.h"
#include "Scenario_FlatbufferScene.h"
#include "PhysicsSystem.h"
#include "CollisionSystem.h"

#include "PeerConfig.h"
#include "WorldSnapshot.h"
#include "ThreadUtils.h"
#include "LoopController.h"
#include "RigidBody.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <thread>
#include <atomic>
#include <memory>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <mutex>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <iostream>

#include "NetworkingSystem.h"
#include "Components.h" // OwnerComponent

 // ============================================================================
 // Replica state storage (net -> sim)
 // ============================================================================
struct ReplicaState
{
    uint32_t objectId = 0;
    glm::vec3 pos{ 0 };
    glm::quat rot{ 1,0,0,0 };
    glm::vec3 linVel{ 0 };
    glm::vec3 angVel{ 0 };
    uint32_t tick = 0;
    double recvTimeSec = 0.0;
};

static constexpr size_t kReplicaSnapshotRingCapacity = 64;

struct ReplicaSnapshotRing
{
    std::array<ReplicaState, kReplicaSnapshotRingCapacity> samples{};
    size_t head = 0;
    size_t count = 0;

    void Clear()
    {
        head = 0;
        count = 0;
    }

    bool Empty() const { return count == 0; }

    const ReplicaState& At(size_t idx) const
    {
        return samples[(head + idx) % kReplicaSnapshotRingCapacity];
    }

    const ReplicaState& Latest() const
    {
        return At(count - 1);
    }

    void Push(const ReplicaState& s)
    {
        if (count > 0)
        {
            const ReplicaState& latest = Latest();
            if (s.tick <= latest.tick)
                return;
        }

        const size_t insertIdx = (head + count) % kReplicaSnapshotRingCapacity;
        samples[insertIdx] = s;

        if (count < kReplicaSnapshotRingCapacity)
        {
            ++count;
        }
        else
        {
            head = (head + 1) % kReplicaSnapshotRingCapacity;
        }
    }
};

static double NowSeconds()
{
    using clock = std::chrono::steady_clock;
    const auto now = clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
}

static glm::quat IntegrateOrientation(const glm::quat& rot, const glm::vec3& angVel, float dt)
{
    const float speed = glm::length(angVel);
    if (speed <= 1e-5f || dt <= 0.0f)
        return rot;

    const glm::vec3 axis = angVel / speed;
    const float angle = speed * dt;
    const glm::quat delta = glm::angleAxis(angle, axis);
    return glm::normalize(delta * rot);
}

static bool SampleReplicaStateAtTime(const ReplicaSnapshotRing& ring,
    double sampleTimeSec, float maxExtrapSec,
    ReplicaState& outState, bool& outUsedExtrapolation)
{
    outUsedExtrapolation = false;
    if (ring.Empty())
        return false;

    if (ring.count == 1)
    {
        outState = ring.Latest();
        return true;
    }

    const ReplicaState& oldest = ring.At(0);
    const ReplicaState& newest = ring.Latest();

    if (sampleTimeSec <= oldest.recvTimeSec)
    {
        outState = oldest;
        return true;
    }

    for (size_t i = 1; i < ring.count; ++i)
    {
        const ReplicaState& b = ring.At(i);
        if (sampleTimeSec <= b.recvTimeSec)
        {
            const ReplicaState& a = ring.At(i - 1);
            const double span = std::max(1e-5, b.recvTimeSec - a.recvTimeSec);
            const float t = (float)((sampleTimeSec - a.recvTimeSec) / span);
            const float clampedT = glm::clamp(t, 0.0f, 1.0f);

            outState = a;
            outState.pos = glm::mix(a.pos, b.pos, clampedT);
            outState.rot = glm::normalize(glm::slerp(a.rot, b.rot, clampedT));
            outState.linVel = glm::mix(a.linVel, b.linVel, clampedT);
            outState.angVel = glm::mix(a.angVel, b.angVel, clampedT);
            outState.tick = (clampedT < 0.5f) ? a.tick : b.tick;
            outState.recvTimeSec = sampleTimeSec;
            return true;
        }
    }

    outUsedExtrapolation = true;
    outState = newest;

    const double dtSec = std::max(0.0, sampleTimeSec - newest.recvTimeSec);
    const float clampedExtrap = std::min((float)dtSec, maxExtrapSec);
    outState.pos = newest.pos + newest.linVel * clampedExtrap;
    outState.rot = IntegrateOrientation(newest.rot, newest.angVel, clampedExtrap);
    outState.recvTimeSec = newest.recvTimeSec + clampedExtrap;
    return true;
}

// ============================================================================
// Shared control state between threads
// ============================================================================
struct SimSharedState
{
    // Application lifetime
    std::atomic<bool> appRunning{ true };

    // Simulation controls (written by render/UI thread, read by sim thread)
    std::atomic<bool>  simRunning{ true };
    std::atomic<bool>  stepOnce{ false };
    std::atomic<bool>  useFixedTimestep{ true };
    std::atomic<float> fixedDt{ 1.0f / 60.0f };
    std::atomic<int>   integratorType{ 0 };   // IntegratorType enum value

    // Scene switching: render thread writes, sim thread reads and acts
    std::atomic<int> requestedSceneIndex{ -1 };

    // Gravity enable flag: controlled globally (and applied by sim thread)
    std::atomic<bool> gravityOn{ true };

    // Local UI (per peer only): display mode
    // 0 = colour by material, 1 = colour by owner
    std::atomic<int> displayMode{ 0 };

    // Current scene instance ID.
 // Incremented only by the peer that initiates a scene change.
 // Receivers copy the value from the incoming scene-change command.
    std::atomic<uint32_t> sceneGeneration{ 1 };
    std::atomic<bool> sceneTransitionActive{ false };
    std::atomic<float> sceneTransitionRemainingSec{ 0.0f };
    // Outgoing GLOBAL UI requests (render thread -> networking thread)
    std::atomic<bool> sendSceneChange{ false };
    std::atomic<int>  sendSceneIndex{ -1 };
    std::atomic<uint32_t> sendSceneGeneration{ 1 };

    std::atomic<bool> sendGravityChange{ false };
    std::atomic<bool> sendGravityEnabled{ true };

    // Incoming GLOBAL UI commands (networking thread -> sim thread)
    std::atomic<bool> pendingSceneChange{ false };
    std::atomic<int>  pendingSceneIndex{ -1 };
    std::atomic<uint32_t> pendingSceneGeneration{ 1 };

    std::atomic<bool> pendingGravityChange{ false };
    std::atomic<bool> pendingGravityEnabled{ true };
    // Shared clear-colour (4 floats; written by UI, read by sim for snapshot)
    float clearColour[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
    std::mutex clearColourMutex;

    // Target Hz — live-adjustable from UI
    std::atomic<float> targetRenderHz{ 60.0f };
    std::atomic<float> targetNetworkHz{ 60.0f };
    std::atomic<float> targetSimHz{ 120.0f };

    // Measured Hz — written by each thread, read by UI
    std::atomic<float> measuredRenderHz{ 0.0f };
    std::atomic<float> measuredNetworkHz{ 0.0f };
    std::atomic<float> measuredSimHz{ 0.0f };
    std::mutex netStatsMutex;
    Net::NetworkStats netStats{};
    // Core indices actually assigned (for UI display)
    int renderCoreAssigned = ThreadUtils::CORE_RENDER;
    int netCoreAssigned = ThreadUtils::CORE_NET_0;
    int simCoreAssigned = ThreadUtils::CORE_SIM_0;

    // ---- Milestone 5: outgoing snapshot (sim -> net) ----
    std::mutex outSnapMutex;
    std::vector<Net::StateSnapshotItem> outOwnedItems;
    uint32_t outTick = 0;
    bool outDirty = false;

    // ---- Milestone 5: incoming replica snapshots (net -> sim) ----
    std::mutex inSnapMutex;
    std::unordered_map<uint32_t, ReplicaSnapshotRing> inReplicaHistory; // objectId -> received snapshot ring

    // ---- Milestone 7: distributed spawns (sim <-> net) ----
    std::mutex outSpawnMutex;
    std::vector<Net::SpawnObjectPayload> outSpawnEvents;

    std::mutex inSpawnMutex;
    std::vector<Net::SpawnObjectPayload> inSpawnEvents;

    // Replica smoothing controls
    std::atomic<bool>  replicaSmoothingEnabled{ true };
    std::atomic<float> replicaInterpDelayMs{ 140.0f };
    std::atomic<float> replicaMaxExtrapolationMs{ 100.0f };
    std::atomic<float> replicaCorrectionThreshold{ 0.75f };

    // Local network impairment tools (affects STATE_SNAPSHOT packets only)
    std::atomic<bool>  netImpairmentEnabled{ false };
    std::atomic<float> netLatencyMs{ 0.0f };
    std::atomic<float> netJitterMs{ 0.0f };
    std::atomic<float> netDropPercent{ 0.0f };

    // Snapshot buffer (sim -> render)
    SnapshotBuffer snapshotBuf;
};

// ============================================================================
// Camera state — owned exclusively by the render / UI thread
// ============================================================================
struct RenderCamera
{
    glm::vec3 position{ 0.0f, 3.0f, -8.0f };
    glm::vec3 rotation{ 0.0f };   // Euler (pitch, yaw, roll) in radians
    float fov = 60.0f;
    float nearClip = 0.1f;
    float farClip = 600.0f;
};

static CameraRenderData BuildCameraRenderData(const RenderCamera& cam, float aspect)
{
    CameraRenderData data{};
    data.position = cam.position;

    const float yaw = cam.rotation.y;
    const float pitch = cam.rotation.x;

    glm::vec3 forward{
        std::cos(pitch) * std::sin(yaw),
        std::sin(pitch),
        std::cos(pitch) * std::cos(yaw)
    };

    data.view = glm::lookAt(cam.position, cam.position + forward, glm::vec3(0, 1, 0));

    const float fovRad = glm::radians(cam.fov);
    data.proj = glm::perspective(fovRad, aspect, cam.nearClip, cam.farClip);
    data.proj[1][1] *= -1.0f; // Vulkan Y-flip
    return data;
}

// ============================================================================
// Owner colour palette (Milestone 4)
// ============================================================================
static glm::vec3 OwnerColor(int ownerId)
{
    if (ownerId < 0) return glm::vec3(0.85f); // owned by all / local copy -> grey

    static const glm::vec3 k[] = {
        {1.0f, 0.2f, 0.2f},  // 0 red
        {0.2f, 1.0f, 0.2f},  // 1 green
        {0.2f, 0.4f, 1.0f},  // 2 blue
        {1.0f, 1.0f, 0.2f},  // 3 yellow
    };
    return k[ownerId % (int)(sizeof(k) / sizeof(k[0]))];
}

// ============================================================================
// Snapshot builder — called by the sim thread after each tick
// ============================================================================
static std::shared_ptr<WorldSnapshot> CaptureSnapshot(
    World& world, uint64_t tickNumber, SimSharedState& shared)
{
    auto snap = std::make_shared<WorldSnapshot>();
    snap->simTickNumber = tickNumber;

    const int displayMode = shared.displayMode.load(std::memory_order_relaxed);

    // Clear colour
    {
        std::lock_guard<std::mutex> lk(shared.clearColourMutex);
        snap->clearColor = glm::vec4(
            shared.clearColour[0], shared.clearColour[1],
            shared.clearColour[2], shared.clearColour[3]);
    }
    snap->ambientLight = glm::vec3(0.02f);

    // Instances (entities with Transform + RenderMesh + Material)
    extern int gForceShading; // defined in Main.cpp
    world.forEach<RenderMeshComponent>([&](Entity e, RenderMeshComponent& meshComp)
        {
            auto* tr = world.getComponent<TransformComponent>(e);
            auto* mat = world.getComponent<MaterialComponent>(e);
            if (!tr || !mat) return;

            glm::vec3 visualScale = tr->scale;

            if (auto* shape = world.getComponent<ShapeComponent>(e))
            {
                std::visit([&](auto&& s)
                    {
                        using T = std::decay_t<decltype(s)>;

                        if constexpr (std::is_same_v<T, SphereShape>)
                        {
                            // sphere.obj base radius = 1.0 (diameter 2.0)
                            visualScale *= glm::vec3(s.radius);
                        }
                        else if constexpr (std::is_same_v<T, CuboidShape>)
                        {
                            // cube.obj base size = 2.0 -> multiply by half to map authored size to desired size
                            visualScale *= (0.5f * s.size);
                        }
                        else if constexpr (std::is_same_v<T, CylinderShape>)
                        {
                            // Placeholder: many scenes use sphere.obj for cylinder visuals.
                            // Scale to bounding dimensions so it isn't oversized.
                            visualScale *= glm::vec3(s.radius, 0.5f * s.height, s.radius);
                        }
                        else if constexpr (std::is_same_v<T, CapsuleShape>)
                        {
                            // Mesh base dimensions (after Blender edit)
                            constexpr float kMeshRadius = 2.0f;
                            constexpr float kMeshHeight = 5.0f; // TOTAL height of the mesh

                            // Desired dimensions from physics shape.
                            // If your CapsuleShape.height is cylinder-height (common):
                            const float desiredRadius = s.radius;
                            const float desiredTotalHeight = s.height + 2.0f * s.radius;

                            // Scale mesh -> desired size
                            visualScale.x *= desiredRadius / kMeshRadius;
                            visualScale.z *= desiredRadius / kMeshRadius;
                            visualScale.y *= desiredTotalHeight / kMeshHeight;
                        }
                        else if constexpr (std::is_same_v<T, PlaneShape>)
                        {
                            // Plane mesh handles its own sizing via Transform scale.
                        }
                    }, shape->shape);
            }

            RenderInstance inst{};
            glm::mat4 M{ 1.0f };
            M = glm::translate(M, tr->position);
            M = glm::rotate(M, tr->rotation.y, glm::vec3(0, 1, 0));
            M = glm::rotate(M, tr->rotation.x, glm::vec3(1, 0, 0));
            M = glm::rotate(M, tr->rotation.z, glm::vec3(0, 0, 1));
            M = glm::scale(M, visualScale);
            inst.model = M;

            inst.meshName = meshComp.meshName;
            inst.textureName = meshComp.textureName;
            if (inst.meshName.empty())
                return;
            inst.shadingModel = mat->shadingModel;

            // Use alpha from MaterialComponent.
            // This requires RenderInstance::diffuseColor to be glm::vec4.
            inst.diffuseColor = glm::vec4(mat->diffuseColor, mat->alpha);

            inst.specularColor = mat->specularColor;
            inst.shininess = mat->shininess;
            inst.castsShadows = mat->castsShadows;
            inst.receivesShadows = mat->receivesShadows;

            // Local UI override: colour by owner
            if (displayMode == 1)
            {
                if (auto* owner = world.getComponent<OwnerComponent>(e))
                {
                    inst.diffuseColor = glm::vec4(OwnerColor(owner->ownerId), mat->alpha);
                    inst.specularColor = glm::vec3(0.05f);
                    inst.shininess = 4.0f;
                }
            }

            if (gForceShading == 0) inst.shadingModel = ShadingModel::Gouraud;
            if (gForceShading == 1) inst.shadingModel = ShadingModel::Phong;
            static int colourDebugCount = 0;

            const bool isFlatbufferObject =
                inst.textureName == "white.jpg" ||
                inst.textureName == "white.png";

            snap->instances.push_back(std::move(inst));

        });

    // Directional lights
    world.forEach<DirectionalLightComponent>([&](Entity, DirectionalLightComponent& light)
        {
            DirectionalLightRenderData d;
            d.direction = light.direction;
            d.color = light.color;
            d.intensity = light.intensity;
            snap->directionalLights.push_back(d);
        });

    // Spark lights
    world.forEach<SparkLightComponent>([&](Entity e, SparkLightComponent& s)
        {
            if (!s.active) return;
            auto* tr = world.getComponent<TransformComponent>(e);
            if (!tr) return;
            SparkLightRenderData out{};
            out.position = tr->position;
            out.radius = s.radius;
            out.color = s.color;
            out.intensity = s.intensity;
            snap->sparkLights.push_back(out);
            if ((int)snap->sparkLights.size() >= 32) return;
        });

    // Particles
    Entity envEntity = INVALID_ENTITY;
    world.forEach<EnvironmentStateComponent>([&](Entity e, EnvironmentStateComponent&)
        {
            if (envEntity == INVALID_ENTITY) envEntity = e;
        });

    if (envEntity != INVALID_ENTITY)
    {
        auto* pool = world.getComponent<ParticlePoolComponent>(envEntity);
        if (pool)
        {
            snap->particles.reserve(pool->particles.size());
            for (const SimParticle& p : pool->particles)
            {
                float lifeT = (p.lifetime > 0.0f) ? (p.age / p.lifetime) : 1.0f;
                lifeT = glm::clamp(lifeT, 0.0f, 1.0f);

                ParticleRenderData out{};
                out.position = p.position;
                switch (p.type)
                {
                case ParticleType::Snow:
                    out.size = 0.06f;
                    out.color = glm::vec4(1, 1, 1, 0.8f * (1.0f - lifeT));
                    break;
                case ParticleType::Rain:
                    out.size = 0.04f;
                    out.color = glm::vec4(0.05f, 0.10f, 0.30f, 0.85f);
                    break;
                case ParticleType::Dust:
                    out.size = 0.15f;
                    out.color = glm::vec4(0.75f, 0.65f, 0.45f, 0.35f * (1.0f - lifeT));
                    break;
                case ParticleType::Fire:
                    out.size = 0.12f;
                    out.color = glm::vec4(1.0f, 0.35f, 0.05f, 0.95f * (1.0f - lifeT));
                    break;
                case ParticleType::Spark:
                    out.size = 0.05f;
                    out.color = glm::vec4(1.0f, 0.9f, 0.4f, 1.0f);
                    break;
                default:
                    out.size = 0.08f;
                    out.color = glm::vec4(1, 0.5f, 0.1f, 0.8f * (1.0f - lifeT));
                    break;
                }
                snap->particles.push_back(out);
            }
        }
    }

    return snap;
}

// ============================================================================
// Simulation thread entry point
// ============================================================================
static void SimulationThreadFunc(SimSharedState& shared, World& world,
    ScenarioManager& scenarios,
    PhysicsSystem& physicsSystem,
    CollisionSystem& collisionSystem, int localPeerId)
{
    // Affinity + name
    const int numCores = ThreadUtils::LogicalCoreCount();
    const int assignedCore = (ThreadUtils::CORE_SIM_0 < numCores)
        ? ThreadUtils::CORE_SIM_0
        : (numCores - 1);
    ThreadUtils::PinCurrentThreadToCore(assignedCore);
    ThreadUtils::SetCurrentThreadName("Sim");
    shared.simCoreAssigned = assignedCore;

    LoopController lc(shared.targetSimHz.load(std::memory_order_relaxed));
    uint64_t tickNumber = 0;
    float    accumulator = 0.0f;
    const float maxDt = 0.25f;

    while (shared.appRunning.load(std::memory_order_relaxed))
    {
        // Live-apply target Hz changes
        lc.setTargetHz(shared.targetSimHz.load(std::memory_order_relaxed));

        float dt = lc.beginFrame();
        shared.measuredSimHz.store(lc.getMeasuredHz(), std::memory_order_relaxed);

        // Clamp to avoid runaway catch-up
        dt = std::min(dt, maxDt);

        // Read controls
        const bool   running = shared.simRunning.load(std::memory_order_relaxed);
        const bool   useFixed = shared.useFixedTimestep.load(std::memory_order_relaxed);
        const float  fdt = shared.fixedDt.load(std::memory_order_relaxed);
        const IntegratorType integ = static_cast<IntegratorType>(
            shared.integratorType.load(std::memory_order_relaxed));

        // stepOnce: consume atomically
        bool doStep = false;
        if (!running)
        {
            bool expected = true;
            doStep = shared.stepOnce.compare_exchange_strong(
                expected, false, std::memory_order_acq_rel);
        }

        physicsSystem.SetIntegrator(integ);

        // ----- Apply GLOBAL commands received from networking thread -----
        if (shared.pendingSceneChange.exchange(false, std::memory_order_acq_rel))
        {
            const int idx = shared.pendingSceneIndex.load(std::memory_order_relaxed);
            shared.requestedSceneIndex.store(idx, std::memory_order_release);
        }

        if (shared.pendingGravityChange.exchange(false, std::memory_order_acq_rel))
        {
            const bool enabled = shared.pendingGravityEnabled.load(std::memory_order_relaxed);
            physicsSystem.SetGravityEnabled(enabled);
            shared.gravityOn.store(enabled, std::memory_order_relaxed);
        }

        // ----- Handle scene switch request from render/UI thread -----
        {
            int desired = shared.requestedSceneIndex.load(std::memory_order_acquire);
            if (desired >= 0 && desired < scenarios.Count())
            {
                if (desired != scenarios.CurrentIndex())
                {
                    const ULONGLONG t = GetTickCount64();
                    std::cout << "[T] SIM applying scene idx=" << desired
                        << " t=" << t << "ms\n";
                    scenarios.SwitchTo(world, desired);
                    accumulator = 0.0f;

                    if (scenarios.Current())
                    {
                        const bool grav = scenarios.Current()->GravityOn();
                        physicsSystem.SetGravityEnabled(grav);
                        shared.gravityOn.store(grav, std::memory_order_relaxed);
                    }


                    // IMPORTANT: clear replication state so old-scene object IDs/states
                    // don't bleed into the newly loaded scene.
                    {
                        std::lock_guard<std::mutex> lk(shared.inSnapMutex);
                        shared.inReplicaHistory.clear();
                    }
                    {
                        std::lock_guard<std::mutex> lk(shared.outSnapMutex);
                        shared.outOwnedItems.clear();
                        shared.outTick = 0;
                        shared.outDirty = false;
                    }
                    {
                        std::lock_guard<std::mutex> lk(shared.inSpawnMutex);
                        shared.inSpawnEvents.clear();
                    }
                    {
                        std::lock_guard<std::mutex> lk(shared.outSpawnMutex);
                        shared.outSpawnEvents.clear();
                    }
                }

                shared.requestedSceneIndex.compare_exchange_strong(
                    desired, -1,
                    std::memory_order_release,
                    std::memory_order_relaxed);
            }
        }

        // ----- Milestone 5+: apply incoming replica snapshots (net -> sim) -----
        {
            std::unordered_map<uint32_t, ReplicaSnapshotRing> history;
            {
                std::lock_guard<std::mutex> lk(shared.inSnapMutex);
                history = shared.inReplicaHistory; // copy to minimize lock time
            }

            const bool smoothingEnabled = shared.replicaSmoothingEnabled.load(std::memory_order_relaxed);
            const float interpDelayMs = shared.replicaInterpDelayMs.load(std::memory_order_relaxed);
            const float maxExtrapMs = shared.replicaMaxExtrapolationMs.load(std::memory_order_relaxed);
            const float correctionThreshold = shared.replicaCorrectionThreshold.load(std::memory_order_relaxed);
            const float smoothingDt = std::max(0.0f, dt);
            const double sampleTimeSec = NowSeconds() - std::max(0.0f, interpDelayMs) * 0.001;
            const float maxExtrapSec = std::max(0.0f, maxExtrapMs) * 0.001f;

            for (auto& kv : history)
            {
                const uint32_t id = kv.first;
                const ReplicaSnapshotRing& ring = kv.second;

                Entity e = (Entity)id;

                auto* phys = world.getComponent<PhysicsComponent>(e);
                auto* tr = world.getComponent<TransformComponent>(e);
                if (!phys || !tr) continue;

                // Don't override authoritative bodies
                auto* owner = world.getComponent<OwnerComponent>(e);

                // Never apply remote snapshots onto objects owned by this local peer.
                if (owner && owner->ownerId == localPeerId - 1)
                    continue;

                // Static/local-all objects should not be driven by network snapshots.
                if (owner && owner->ownerId < 0)
                    continue;

                ReplicaState target{};
                bool usedExtrapolation = false;
                if (!SampleReplicaStateAtTime(ring, sampleTimeSec, maxExtrapSec, target, usedExtrapolation))
                    continue;

                /*std::cout << "[SnapshotApplyDebug] localPeer="
                    << localPeerId
                    << " entity=" << e
                    << " ownerId=" << (owner ? owner->ownerId : -999)
                    << " currentPos=("
                    << phys->body.Position().x << ","
                    << phys->body.Position().y << ","
                    << phys->body.Position().z << ")"
                    << " targetPos=("
                    << target.pos.x << ","
                    << target.pos.y << ","
                    << target.pos.z << ")"
                    << " targetVel=("
                    << target.linVel.x << ","
                    << target.linVel.y << ","
                    << target.linVel.z << ")"
                    << " smoothing=" << (smoothingEnabled ? "yes" : "no")
                    << "\n";
                    */
                
                    phys->body.SetPosition(target.pos);
                    phys->body.SetOrientation(target.rot);
                    phys->body.SetLinearVelocity(target.linVel);
                    phys->body.SetAngularVelocity(target.angVel);
                    tr->position = target.pos;
                    tr->rotation = glm::eulerAngles(target.rot);
                    continue;
                

                const glm::vec3 currentPos = phys->body.Position();
                const glm::quat currentRot = phys->body.Orientation();
                const glm::vec3 currentLinVel = phys->body.LinearVelocity();
                const glm::vec3 currentAngVel = phys->body.AngularVelocity();

                const float posError = glm::length(target.pos - currentPos);
                const bool largeError = posError > correctionThreshold;

                float spring = largeError ? 7.0f : 16.0f;
                if (usedExtrapolation)
                    spring *= 0.75f;

                const float blend = glm::clamp(spring * smoothingDt, 0.0f, 1.0f);

                const glm::vec3 smoothPos = glm::mix(currentPos, target.pos, blend);
                const glm::quat smoothRot = glm::normalize(glm::slerp(currentRot, target.rot, blend));
                const glm::vec3 smoothLinVel = glm::mix(currentLinVel, target.linVel, blend);
                const glm::vec3 smoothAngVel = glm::mix(currentAngVel, target.angVel, blend);

                phys->body.SetPosition(smoothPos);
                phys->body.SetOrientation(smoothRot);
                phys->body.SetLinearVelocity(smoothLinVel);
                phys->body.SetAngularVelocity(smoothAngVel);

                tr->position = smoothPos;
                tr->rotation = glm::eulerAngles(smoothRot);
            }
        }

        if (Scenario* s = scenarios.Current())
        {
            // ------------------------------------------------------------
            // Scene transition pause.
            //
            // During this pause:
            // - do not apply incoming spawns
            // - do not update spawners
            // - do not step physics
            // - do not generate outgoing spawn events
            //
            // This gives remote peers time to load the same scene/generation
            // before owner peers start spawning and simulating objects.
            // ------------------------------------------------------------

            if (shared.sceneTransitionActive.load(std::memory_order_acquire))
            {
                float remaining =
                    shared.sceneTransitionRemainingSec.load(std::memory_order_relaxed);

                // Count down using the real sim-thread frame delta.
                // Do this once per outer sim loop frame, not once per fixed physics step.
                remaining -= dt;

                std::cout << "[TRANSITION TICK] remaining="
                    << remaining
                    << " dt="
                    << dt
                    << "\n";

                if (remaining <= 0.0f)
                {
                    shared.sceneTransitionRemainingSec.store(0.0f, std::memory_order_relaxed);
                    shared.sceneTransitionActive.store(false, std::memory_order_release);

                    // Clear any object traffic that slipped through during transition.
                    {
                        std::lock_guard<std::mutex> lk(shared.inSpawnMutex);
                        shared.inSpawnEvents.clear();
                    }

                    {
                        std::lock_guard<std::mutex> lk(shared.outSpawnMutex);
                        shared.outSpawnEvents.clear();
                    }

                    {
                        std::lock_guard<std::mutex> lk(shared.inSnapMutex);
                        shared.inReplicaHistory.clear();
                    }

                    {
                        std::lock_guard<std::mutex> lk(shared.outSnapMutex);
                        shared.outOwnedItems.clear();
                        shared.outTick = 0;
                        shared.outDirty = false;
                    }

                    accumulator = 0.0f;

                    std::cout << "[SCENE START] gen="
                        << shared.sceneGeneration.load(std::memory_order_acquire)
                        << "\n";
                }
                else
                {
                    shared.sceneTransitionRemainingSec.store(
                        remaining,
                        std::memory_order_relaxed);

                    // Skip the rest of this simulation frame.
                    // Important: call endFrame before continue so the loop controller
                    // still sleeps/throttles instead of spinning and burning the delay instantly.
                    lc.endFrame();
                    continue;
                }
            }

            const uint32_t currentSceneGeneration =
                shared.sceneGeneration.load(std::memory_order_acquire);

            // ------------------------------------------------------------
            // Apply incoming network spawns only after transition is complete.
            // ------------------------------------------------------------

            if (auto* fb = dynamic_cast<Scenario_FlatbufferScene*>(s))
            {
                std::vector<Net::SpawnObjectPayload> incomingSpawns;

                {
                    std::lock_guard<std::mutex> lk(shared.inSpawnMutex);
                    incomingSpawns.swap(shared.inSpawnEvents);
                }

                for (const auto& payload : incomingSpawns)
                {
                    if (payload.sceneGeneration != currentSceneGeneration)
                        continue;

                    fb->SpawnFromNetworkEvent(world, payload);
                }
            }

            // ------------------------------------------------------------
            // Update simulation.
            // ------------------------------------------------------------

            if (useFixed)
            {
                if (running)
                    accumulator += dt;
                else if (doStep)
                    accumulator += fdt;

                while (accumulator >= fdt)
                {
                    s->Update(world, fdt, currentSceneGeneration);
                    physicsSystem.Update(world, fdt);
                    collisionSystem.Update(world);

                    accumulator -= fdt;
                    ++tickNumber;
                }
            }
            else
            {
                if (running || doStep)
                {
                    const float stepDt = running ? dt : fdt;

                    s->Update(world, stepDt, currentSceneGeneration);
                    physicsSystem.Update(world, stepDt);
                    collisionSystem.Update(world);

                    ++tickNumber;
                }
            }

            // ------------------------------------------------------------
            // Collect locally generated spawns only after transition is complete.
            // ------------------------------------------------------------

            if (auto* fb = dynamic_cast<Scenario_FlatbufferScene*>(s))
            {
                Net::SpawnObjectPayload spawnPayload{};
                std::vector<Net::SpawnObjectPayload> generated;

                while (fb->PopPendingSpawn(spawnPayload))
                {
                    if (spawnPayload.sceneGeneration != currentSceneGeneration)
                        continue;

                    generated.push_back(spawnPayload);
                }

                if (!generated.empty())
                {
                    std::lock_guard<std::mutex> lk(shared.outSpawnMutex);
                    shared.outSpawnEvents.insert(
                        shared.outSpawnEvents.end(),
                        generated.begin(),
                        generated.end());
                }
            }
        }

        // ----- Milestone 5: build outgoing STATE_SNAPSHOT (owned dynamics) -----
        {
            std::vector<Net::StateSnapshotItem> items;
            items.reserve(4096);

            world.forEach<PhysicsComponent>([&](Entity e, PhysicsComponent& phys)
                {
                    auto* owner = world.getComponent<OwnerComponent>(e);

                    // Only simulated owned objects should be sent by this peer.
                    // ownerId is 0-based, localPeerId is 1-based.
                    if (!owner)
                        return;

                    if (owner->ownerId != localPeerId - 1)
                        return;

                    if (!phys.body.IsDynamic())
                    {
                        std::cout << "[SnapshotBuildWarn] localPeer="
                            << localPeerId
                            << " entity=" << e
                            << " ownerId=" << owner->ownerId
                            << " ownedButNotDynamic=1"
                            << "\n";
                        return;
                    }

                    Net::StateSnapshotItem it{};
                    it.objectId = (uint32_t)e;

                    const glm::vec3 p = phys.body.Position();
                    const glm::quat q = phys.body.Orientation();
                    const glm::vec3 v = phys.body.LinearVelocity();
                    const glm::vec3 w = phys.body.AngularVelocity();

                   /* std::cout << "[SnapshotBuildDebug] localPeer="
                        << localPeerId
                        << " entity=" << e
                        << " ownerId=" << owner->ownerId
                        << " pos=(" << p.x << "," << p.y << "," << p.z << ")"
                        << " vel=(" << v.x << "," << v.y << "," << v.z << ")"
                        << "\n";
                       */
                    it.pos = { p.x, p.y, p.z };
                    it.rot = { q.x, q.y, q.z, q.w };
                    it.linVel = { v.x, v.y, v.z };
                    it.angVel = { w.x, w.y, w.z };

                    items.push_back(it);
                });

            std::lock_guard<std::mutex> lk(shared.outSnapMutex);
            shared.outOwnedItems = std::move(items);
            shared.outTick = (uint32_t)tickNumber;
            shared.outDirty = true;
        }

        // Publish render snapshot every tick
        shared.snapshotBuf.publish(CaptureSnapshot(world, tickNumber, shared));

        lc.endFrame();
    }
}

// ============================================================================
// Networking thread entry point
// ============================================================================
static void NetworkingThreadFunc(SimSharedState& shared, const Net::PeerConfig& cfg)
{
    using namespace Net;
    const int localPeerId = cfg.peer_id;

    // ---- Thread setup ----
    const int numCores = ThreadUtils::LogicalCoreCount();
    const int assignedCore = (ThreadUtils::CORE_NET_0 < numCores)
        ? ThreadUtils::CORE_NET_0
        : (numCores - 1);

    ThreadUtils::PinCurrentThreadToCore(assignedCore);
    ThreadUtils::SetCurrentThreadName("Net");
    shared.netCoreAssigned = assignedCore;

    NetworkingSystem net;
    if (!net.Init(cfg))
        return;

    LoopController lc(shared.targetNetworkHz.load(std::memory_order_relaxed));

    // Snapshot send throttling.
    float snapshotSendAccum = 0.0f;
    constexpr float SNAPSHOT_SEND_HZ = 20.0f;
    constexpr float SNAPSHOT_SEND_INTERVAL = 1.0f / SNAPSHOT_SEND_HZ;

    while (shared.appRunning.load(std::memory_order_relaxed))
    {
        lc.setTargetHz(shared.targetNetworkHz.load(std::memory_order_relaxed));

        float dt = lc.beginFrame();
        shared.measuredNetworkHz.store(lc.getMeasuredHz(), std::memory_order_relaxed);

        // Keep networking layer aligned with the currently valid scene generation.
        net.SetCurrentSceneGeneration(
            shared.sceneGeneration.load(std::memory_order_acquire));

        Net::SnapshotImpairmentSettings impair{};
        impair.enabled = shared.netImpairmentEnabled.load(std::memory_order_relaxed);
        impair.latencyMs = shared.netLatencyMs.load(std::memory_order_relaxed);
        impair.jitterMs = shared.netJitterMs.load(std::memory_order_relaxed);
        impair.dropPercent = shared.netDropPercent.load(std::memory_order_relaxed);
        net.SetSnapshotImpairment(impair);

        // ------------------------------------------------------------
        // High-priority outgoing global commands.
        // These must happen before net.Update(dt), so they are not stuck
        // behind snapshot/resend processing.
        // ------------------------------------------------------------

        if (shared.sendSceneChange.exchange(false, std::memory_order_acq_rel))
        {
            const int idx =
                shared.sendSceneIndex.load(std::memory_order_relaxed);

            const uint32_t generation =
                shared.sendSceneGeneration.load(std::memory_order_acquire);

            // Old scene object traffic is no longer valid.
            net.ClearSceneObjectTraffic();
            net.PauseSnapshotTraffic(0.25f);
            net.SetCurrentSceneGeneration(generation);

            net.SendSceneChange(idx, generation);
        }

        if (shared.sendGravityChange.exchange(false, std::memory_order_acq_rel))
        {
            const bool enabled =
                shared.sendGravityEnabled.load(std::memory_order_relaxed);

            net.PauseSnapshotTraffic(0.10f);
            net.SendGravityEnabled(enabled);
        }

        // ------------------------------------------------------------
        // Process sockets / ACKs / reliable resends / delayed snapshots.
        // ------------------------------------------------------------

        net.Update(dt);

        {
            std::lock_guard<std::mutex> lk(shared.netStatsMutex);
            shared.netStats = net.GetStats();
        }

        // ------------------------------------------------------------
        // Send SPAWN_OBJECT packets.
        //
        // Important: do not send stale spawn events from an older scene
        // generation.
        // ------------------------------------------------------------

        if (shared.sceneTransitionActive.load(std::memory_order_acquire))
        {
            std::lock_guard<std::mutex> lk(shared.outSpawnMutex);
            shared.outSpawnEvents.clear();
        }
        else
        {
            std::vector<SpawnObjectPayload> pending;

            {
                std::lock_guard<std::mutex> lk(shared.outSpawnMutex);
                if (!shared.outSpawnEvents.empty())
                    pending.swap(shared.outSpawnEvents);
            }

            const uint32_t currentGeneration =
                shared.sceneGeneration.load(std::memory_order_acquire);

            for (const auto& spawn : pending)
            {
                if (spawn.sceneGeneration != currentGeneration)
                    continue;

                net.SendSpawnObject(spawn);
            }
        }

        // ------------------------------------------------------------
        // Receive incoming global commands and forward to sim thread.
        // ------------------------------------------------------------

        GlobalCommandPayload cmd{};
        while (net.PopReceivedGlobalCommand(cmd))
        {
            const auto type = (GlobalCommandType)cmd.commandType;

            const ULONGLONG t = GetTickCount64();
            std::cout << "[T] NET popped GLOBAL cmd scene=" << cmd.sceneIndex
                << " generation=" << cmd.sceneGeneration
                << " t=" << t << "ms\n";

            if (type == GlobalCommandType::SceneChange)
            {
                // Use the sender's generation.
                // Do NOT increment locally on received scene changes.
                shared.sceneGeneration.store(
                    cmd.sceneGeneration,
                    std::memory_order_release);

                shared.pendingSceneGeneration.store(
                    cmd.sceneGeneration,
                    std::memory_order_release);

                shared.pendingSceneIndex.store(
                    cmd.sceneIndex,
                    std::memory_order_relaxed);

                shared.pendingSceneChange.store(
                    true,
                    std::memory_order_release);

                // Keep network layer immediately aligned too.
                net.SetCurrentSceneGeneration(cmd.sceneGeneration);

                // Clear any outgoing old-scene object data.
                {
                    std::lock_guard<std::mutex> lk(shared.outSnapMutex);
                    shared.outOwnedItems.clear();
                    shared.outTick = 0;
                    shared.outDirty = false;
                }

                {
                    std::lock_guard<std::mutex> lk(shared.outSpawnMutex);
                    shared.outSpawnEvents.clear();
                }

                // Also clear incoming old-scene data.
                {
                    std::lock_guard<std::mutex> lk(shared.inSnapMutex);
                    shared.inReplicaHistory.clear();
                }

                {
                    std::lock_guard<std::mutex> lk(shared.inSpawnMutex);
                    shared.inSpawnEvents.clear();
                }

                net.ClearSceneObjectTraffic();
                net.PauseSnapshotTraffic(0.25f);
            }
            else if (type == GlobalCommandType::GravityOnOff)
            {
                shared.pendingGravityEnabled.store(
                    cmd.gravityEnabled != 0,
                    std::memory_order_relaxed);

                shared.pendingGravityChange.store(
                    true,
                    std::memory_order_release);
            }
        }

        // ------------------------------------------------------------
        // Send STATE_SNAPSHOT packets.
        // ------------------------------------------------------------

        snapshotSendAccum += dt;

        if (shared.sceneTransitionActive.load(std::memory_order_acquire))
        {
            // Drop any old outgoing snapshot data generated during transition.
            std::lock_guard<std::mutex> lk(shared.outSnapMutex);
            shared.outOwnedItems.clear();
            shared.outTick = 0;
            shared.outDirty = false;

            // Avoid immediately sending a snapshot burst when transition ends.
            snapshotSendAccum = 0.0f;
        }
        else if (snapshotSendAccum >= SNAPSHOT_SEND_INTERVAL)
        {
            snapshotSendAccum -= SNAPSHOT_SEND_INTERVAL;

            std::vector<StateSnapshotItem> items;
            uint32_t tick = 0;
            bool dirty = false;

            {
                std::lock_guard<std::mutex> lk(shared.outSnapMutex);
                dirty = shared.outDirty;

                if (dirty)
                {
                    items = shared.outOwnedItems;
                    tick = shared.outTick;
                    shared.outDirty = false;
                }
            }

            if (dirty && !items.empty())
            {
                const uint32_t generation =
                    shared.sceneGeneration.load(std::memory_order_acquire);

                net.SendStateSnapshot(
                    tick,
                    generation,
                    items.data(),
                    (uint32_t)items.size());
            }
        }

        // ------------------------------------------------------------
        // Receive STATE_SNAPSHOT packets.
        // NetworkingSystem already filters by sceneGeneration, but we
        // also only store what it gives us here.
        // ------------------------------------------------------------

        {
            std::vector<StateSnapshotItem> items;
            uint32_t tick = 0;

            while (net.PopReceivedStateSnapshot(items, tick))
            {
                std::lock_guard<std::mutex> lk(shared.inSnapMutex);
                const double nowSec = NowSeconds();

                for (const auto& it : items)
                {
                    ReplicaState st{};
                    st.objectId = it.objectId;
                    st.pos = glm::vec3(it.pos.x, it.pos.y, it.pos.z);
                    st.rot = glm::quat(it.rot.w, it.rot.x, it.rot.y, it.rot.z);
                    st.linVel = glm::vec3(it.linVel.x, it.linVel.y, it.linVel.z);
                    st.angVel = glm::vec3(it.angVel.x, it.angVel.y, it.angVel.z);
                    st.tick = tick;
                    st.recvTimeSec = nowSec;

                    auto& ring = shared.inReplicaHistory[it.objectId];
                    ring.Push(st);
                }
            }
        }

        // ------------------------------------------------------------
        // Receive SPAWN_OBJECT packets and forward to sim thread.
        // NetworkingSystem already filters by sceneGeneration, but we
        // filter again here for safety.
        // ------------------------------------------------------------

        {
            SpawnObjectPayload payload{};
            std::vector<SpawnObjectPayload> received;

            const uint32_t currentGeneration =
                shared.sceneGeneration.load(std::memory_order_acquire);

            while (net.PopReceivedSpawnObject(payload))
            {
                if (payload.sceneGeneration != currentGeneration)
                    continue;

                received.push_back(payload);
            }

            if (!received.empty())
            {
                std::lock_guard<std::mutex> lk(shared.inSpawnMutex);
                shared.inSpawnEvents.insert(
                    shared.inSpawnEvents.end(),
                    received.begin(),
                    received.end());
            }
        }

        lc.endFrame();
    }
    net.Shutdown();
}

// ============================================================================
// RunSandbox — main / render thread
// ============================================================================
int RunSandbox(GLFWwindow* window, Renderer& renderer, const Net::PeerConfig& cfg)
{
    const int localPeerId = cfg.peer_id;

    World world;
    PhysicsSystem physicsSystem;
    physicsSystem.SetLocalPeerId(localPeerId);

    CollisionSystem  collisionSystem;
    ScenarioManager  scenarios;

    scenarios.Add(std::make_unique<Scenario_PrimitiveScene>());
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/newtonsCradle.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/bouncingBalls.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/piston.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/sphereSpawners.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/tumbler.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/elasticityDemo.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/gridDropSpheres.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/wavePlatforms.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/weightedCradle.bin"));


    for (int i = 0; i < scenarios.Count(); ++i)
    {
        if (auto* fb = dynamic_cast<Scenario_FlatbufferScene*>(scenarios.Get(i)))
            fb->SetLocalPeerId(localPeerId);
    }

    if (scenarios.Count() > 0)
        scenarios.SwitchTo(world, 0);

    SimSharedState shared;
    shared.targetRenderHz.store(cfg.render_hz, std::memory_order_relaxed);
    shared.targetNetworkHz.store(cfg.network_hz, std::memory_order_relaxed);
    shared.targetSimHz.store(cfg.simulation_hz, std::memory_order_relaxed);

    // Initialise gravity from the initial scene
    if (scenarios.Current())
    {
        const bool grav = scenarios.Current()->GravityOn();
        physicsSystem.SetGravityEnabled(grav);
        shared.gravityOn.store(grav, std::memory_order_relaxed);
    }

    // Initialise clear colour from World
    {
        std::lock_guard<std::mutex> lk(shared.clearColourMutex);
        const float* cc = world.ClearColour();
        shared.clearColour[0] = cc[0];
        shared.clearColour[1] = cc[1];
        shared.clearColour[2] = cc[2];
        shared.clearColour[3] = cc[3];
    }

    // Extract initial camera state from the Overview camera entity
    RenderCamera renderCamera;
    world.forEach<CameraRoleComponent>([&](Entity e, CameraRoleComponent& r)
        {
            if (r.role == CameraRole::Overview)
            {
                if (auto* tr = world.getComponent<TransformComponent>(e))
                {
                    renderCamera.position = tr->position;
                    renderCamera.rotation = tr->rotation;
                }
                if (auto* cam = world.getComponent<CameraComponent>(e))
                {
                    renderCamera.fov = cam->fov;
                    renderCamera.nearClip = cam->nearClip;
                    renderCamera.farClip = cam->farClip;
                }
            }
        });

    // Pin main (render) thread
    const int numCores = ThreadUtils::LogicalCoreCount();
    const int renderCore = (ThreadUtils::CORE_RENDER < numCores)
        ? ThreadUtils::CORE_RENDER : (numCores - 1);
    ThreadUtils::PinCurrentThreadToCore(renderCore);
    ThreadUtils::SetCurrentThreadName("Render");
    shared.renderCoreAssigned = renderCore;

    // Produce an initial snapshot so the renderer doesn't start blank
    shared.snapshotBuf.publish(CaptureSnapshot(world, 0, shared));

    // Start background threads
    std::thread simThread(
        SimulationThreadFunc,
        std::ref(shared),
        std::ref(world),
        std::ref(scenarios),
        std::ref(physicsSystem),
        std::ref(collisionSystem),
        localPeerId
    );

    std::thread netThread(NetworkingThreadFunc, std::ref(shared), std::cref(cfg));

    // Render / UI loop state
    LoopController renderLC(shared.targetRenderHz.load(std::memory_order_relaxed));
    CameraRole activeCameraMode = CameraRole::Overview;
    const float maxFrameDt = 0.25f;

    float displayRenderHz = 0.0f;
    float displayNetHz = 0.0f;
    float displaySimHz = 0.0f;

    float uiRenderHz = cfg.render_hz;
    float uiNetHz = cfg.network_hz;
    float uiSimHz = cfg.simulation_hz;

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        renderLC.setTargetHz(shared.targetRenderHz.load(std::memory_order_relaxed));
        float dt = renderLC.beginFrame();
        shared.measuredRenderHz.store(renderLC.getMeasuredHz(), std::memory_order_relaxed);
        dt = std::min(dt, maxFrameDt);

        // Camera movement (render thread only)
        if (!ImGui::GetIO().WantCaptureKeyboard)
        {
            const float lookSpeed = 1.5f;
            if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) renderCamera.rotation.y -= lookSpeed * dt;
            if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)  renderCamera.rotation.y += lookSpeed * dt;
            if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)    renderCamera.rotation.x += lookSpeed * dt;
            if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)  renderCamera.rotation.x -= lookSpeed * dt;

            const float pitchLimit = glm::radians(89.0f);
            renderCamera.rotation.x = glm::clamp(renderCamera.rotation.x, -pitchLimit, pitchLimit);

            const float moveSpeed = 10.0f;
            glm::vec3 moveLocal(0.0f);
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) moveLocal.z += 1.0f;
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) moveLocal.z -= 1.0f;
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) moveLocal.x -= 1.0f;
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) moveLocal.x += 1.0f;
            if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) moveLocal.y -= 1.0f;
            if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) moveLocal.y += 1.0f;

            if (moveLocal.x != 0.0f || moveLocal.y != 0.0f || moveLocal.z != 0.0f)
            {
                moveLocal = glm::normalize(moveLocal);
                float yaw = renderCamera.rotation.y;
                glm::vec3 forward(std::sin(yaw), 0.0f, std::cos(yaw));
                glm::vec3 right(forward.z, 0.0f, -forward.x);
                glm::vec3 up(0.0f, 1.0f, 0.0f);
                renderCamera.position += (right * moveLocal.x + up * moveLocal.y + forward * moveLocal.z) * (moveSpeed * dt);
            }
        }

        // Snapshot: consume latest from sim thread
        std::shared_ptr<WorldSnapshot> snap = shared.snapshotBuf.consume();

        // ImGui
        renderer.BeginImGuiFrame();

        displayRenderHz = shared.measuredRenderHz.load(std::memory_order_relaxed);
        displayNetHz = shared.measuredNetworkHz.load(std::memory_order_relaxed);
        displaySimHz = shared.measuredSimHz.load(std::memory_order_relaxed);

        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("Simulation"))
            {
                bool simRunning = shared.simRunning.load(std::memory_order_relaxed);
                if (ImGui::MenuItem(simRunning ? "Pause" : "Start"))
                    shared.simRunning.store(!simRunning, std::memory_order_relaxed);

                if (ImGui::MenuItem("Step Once"))
                    shared.stepOnce.store(true, std::memory_order_relaxed);

                ImGui::Separator();

                bool useFixed = shared.useFixedTimestep.load(std::memory_order_relaxed);
                if (ImGui::Checkbox("Use Fixed Timestep", &useFixed))
                    shared.useFixedTimestep.store(useFixed, std::memory_order_relaxed);

                float fdt = shared.fixedDt.load(std::memory_order_relaxed);
                float oldFdt = fdt;
                ImGui::InputFloat("Fixed dt (s)", &fdt, 0.001f, 0.01f, "%.4f");
                fdt = std::max(0.0001f, std::min(fdt, 0.1f));
                if (fdt != oldFdt)
                    shared.fixedDt.store(fdt, std::memory_order_relaxed);

                ImGui::Text("~ %.1f Hz", 1.0f / fdt);
                ImGui::Separator();

                const char* integrators[] = { "Euler", "Semi-Implicit Euler" };
                int integ = shared.integratorType.load(std::memory_order_relaxed);
                if (ImGui::Combo("Integrator", &integ, integrators, IM_ARRAYSIZE(integrators)))
                    shared.integratorType.store(integ, std::memory_order_relaxed);

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("View"))
            {
                {
                    std::lock_guard<std::mutex> lk(shared.clearColourMutex);
                    ImGui::ColorEdit3("Background", shared.clearColour);
                }

                ImGui::SeparatorText("Local Display");
                int mode = shared.displayMode.load(std::memory_order_relaxed);
                if (ImGui::RadioButton("Colour by Material", mode == 0)) mode = 0;
                if (ImGui::RadioButton("Colour by Owner", mode == 1)) mode = 1;
                shared.displayMode.store(mode, std::memory_order_relaxed);

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Camera"))
            {
                bool isOverview = (activeCameraMode == CameraRole::Overview);
                bool isNavigation = (activeCameraMode == CameraRole::Navigation);

                if (ImGui::MenuItem("Overview", nullptr, isOverview))
                {
                    activeCameraMode = CameraRole::Overview;
                    renderCamera.position = glm::vec3(0.0f, 25.0f, 0.0f);
                    renderCamera.rotation = glm::radians(glm::vec3(-90.0f, 0.0f, 0.0f));
                }
                if (ImGui::MenuItem("Navigation", nullptr, isNavigation))
                {
                    activeCameraMode = CameraRole::Navigation;
                    renderCamera.position = glm::vec3(0.0f, 3.0f, -8.0f);
                    renderCamera.rotation = glm::radians(glm::vec3(-15.0f, 0.0f, 0.0f));
                }

                ImGui::Separator();
                ImGui::DragFloat3("Position", &renderCamera.position.x, 0.1f);
                ImGui::DragFloat3("Rotation (rad)", &renderCamera.rotation.x, 0.02f);
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Scene"))
            {
                ImGui::SeparatorText("Switch Scene");
                ImGui::TextDisabled("(Global: all peers will switch)");
                ImGui::Separator();

                const int currentIdx = scenarios.CurrentIndex();
                for (int si = 0; si < scenarios.Count(); ++si)
                {
                    const bool selected = (si == currentIdx);
                    if (ImGui::MenuItem(scenarios.Get(si)->Name(), nullptr, selected))
                    {
                        if (si != currentIdx)
                        {
                            const ULONGLONG t = GetTickCount64();

                            const uint32_t newGeneration =
                                shared.sceneGeneration.fetch_add(1, std::memory_order_acq_rel) + 1;

                            std::cout << "[T] UI scene click idx=" << si
                                << " gen=" << newGeneration
                                << " t=" << t << "ms\n";

                            // Local scene switch invalidates old outgoing/incoming network state.
                            {
                                std::lock_guard<std::mutex> lk(shared.outSnapMutex);
                                shared.outOwnedItems.clear();
                                shared.outTick = 0;
                                shared.outDirty = false;
                            }

                            {
                                std::lock_guard<std::mutex> lk(shared.outSpawnMutex);
                                shared.outSpawnEvents.clear();
                            }

                            {
                                std::lock_guard<std::mutex> lk(shared.inSnapMutex);
                                shared.inReplicaHistory.clear();
                            }

                            {
                                std::lock_guard<std::mutex> lk(shared.inSpawnMutex);
                                shared.inSpawnEvents.clear();
                            }

                            // Start a short transition pause.
                            // The scene can load locally, but sim/spawners/snapshots should not run yet.
                            shared.sceneTransitionActive.store(true, std::memory_order_release);
                            shared.sceneTransitionRemainingSec.store(0.10f, std::memory_order_release);
                            std::cout << "[TRANSITION BEGIN UI] delay="
                                << shared.sceneTransitionRemainingSec.load(std::memory_order_acquire)

                                << "\n";
                            // Apply locally immediately.
                            shared.requestedSceneIndex.store(si, std::memory_order_release);

                            // Send globally with the same generation.
                            shared.sendSceneGeneration.store(newGeneration, std::memory_order_release);
                            shared.sendSceneIndex.store(si, std::memory_order_relaxed);
                            shared.sendSceneChange.store(true, std::memory_order_release);
                        }
                    }
                }
            

                ImGui::Separator();

                bool grav = shared.gravityOn.load(std::memory_order_relaxed);
                ImGui::Text("Gravity: %s", grav ? "ON" : "OFF");
                if (ImGui::Checkbox("Gravity Enabled (GLOBAL)", &grav))
                {
                    // Apply locally (sim thread will apply next tick)
                    shared.pendingGravityEnabled.store(grav, std::memory_order_relaxed);
                    shared.pendingGravityChange.store(true, std::memory_order_release);

                    // Send globally
                    shared.sendGravityEnabled.store(grav, std::memory_order_relaxed);
                    shared.sendGravityChange.store(true, std::memory_order_release);
                }

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Concurrency"))
            {
                ImGui::SeparatorText("Target Frequencies");

                if (ImGui::SliderFloat("Render Hz", &uiRenderHz, 0.0f, 300.0f, "%.0f"))
                    shared.targetRenderHz.store(uiRenderHz, std::memory_order_relaxed);
                if (ImGui::SliderFloat("Network Hz", &uiNetHz, 0.0f, 120.0f, "%.0f"))
                    shared.targetNetworkHz.store(uiNetHz, std::memory_order_relaxed);
                if (ImGui::SliderFloat("Sim Hz", &uiSimHz, 0.0f, 500.0f, "%.0f"))
                    shared.targetSimHz.store(uiSimHz, std::memory_order_relaxed);

                ImGui::TextDisabled("(0 = uncapped)");
                ImGui::Separator();

                ImGui::SeparatorText("Measured Frequencies");
                ImGui::Text("Render  : %6.1f Hz", displayRenderHz);
                ImGui::Text("Network : %6.1f Hz", displayNetHz);
                ImGui::Text("Sim     : %6.1f Hz", displaySimHz);
                ImGui::Separator();

                ImGui::SeparatorText("Replica Smoothing");
                {
                    bool smoothOn = shared.replicaSmoothingEnabled.load(std::memory_order_relaxed);
                    if (ImGui::Checkbox("Enable Smoothing", &smoothOn))
                        shared.replicaSmoothingEnabled.store(smoothOn, std::memory_order_relaxed);

                    float interpDelayMs = shared.replicaInterpDelayMs.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Interpolation Delay (ms)", &interpDelayMs, 60.0f, 250.0f, "%.0f"))
                        shared.replicaInterpDelayMs.store(interpDelayMs, std::memory_order_relaxed);

                    float extrapMs = shared.replicaMaxExtrapolationMs.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Max Extrapolation (ms)", &extrapMs, 0.0f, 200.0f, "%.0f"))
                        shared.replicaMaxExtrapolationMs.store(extrapMs, std::memory_order_relaxed);

                    float correctionThreshold = shared.replicaCorrectionThreshold.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Large Error Threshold (m)", &correctionThreshold, 0.1f, 3.0f, "%.2f"))
                        shared.replicaCorrectionThreshold.store(correctionThreshold, std::memory_order_relaxed);
                }
                ImGui::Separator();

                ImGui::SeparatorText("Network Impairment (Local)");
                {
                    bool impairOn = shared.netImpairmentEnabled.load(std::memory_order_relaxed);
                    if (ImGui::Checkbox("Enable Snapshot Impairment", &impairOn))
                        shared.netImpairmentEnabled.store(impairOn, std::memory_order_relaxed);

                    float latencyMs = shared.netLatencyMs.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Latency (ms)", &latencyMs, 0.0f, 300.0f, "%.0f"))
                        shared.netLatencyMs.store(latencyMs, std::memory_order_relaxed);

                    float jitterMs = shared.netJitterMs.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Latency Jitter (ms)", &jitterMs, 0.0f, 150.0f, "%.0f"))
                        shared.netJitterMs.store(jitterMs, std::memory_order_relaxed);

                    float dropPct = shared.netDropPercent.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Packet Drop (%)", &dropPct, 0.0f, 80.0f, "%.0f"))
                        shared.netDropPercent.store(dropPct, std::memory_order_relaxed);

                    if (ImGui::Button("Preset: Stable Network"))
                    {
                        shared.netImpairmentEnabled.store(false, std::memory_order_relaxed);
                        shared.netLatencyMs.store(0.0f, std::memory_order_relaxed);
                        shared.netJitterMs.store(0.0f, std::memory_order_relaxed);
                        shared.netDropPercent.store(0.0f, std::memory_order_relaxed);
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Preset: 100ms +/-50ms, 20% drop"))
                    {
                        shared.netImpairmentEnabled.store(true, std::memory_order_relaxed);
                        shared.netLatencyMs.store(100.0f, std::memory_order_relaxed);
                        shared.netJitterMs.store(50.0f, std::memory_order_relaxed);
                        shared.netDropPercent.store(20.0f, std::memory_order_relaxed);
                    }
                }
                ImGui::Separator();

                ImGui::SeparatorText("Network Debug Stats");
                {
                    Net::NetworkStats netStats{};
                    {
                        std::lock_guard<std::mutex> lk(shared.netStatsMutex);
                        netStats = shared.netStats;
                    }

                    ImGui::Text("Control port              : %u", (unsigned)cfg.control_bind_port);
                    ImGui::Text("Snapshot port             : %u", (unsigned)cfg.snapshot_bind_port);
                    ImGui::Text("Control packets received  : %u", netStats.controlPacketsReceived);
                    ImGui::Text("Snapshot packets sent     : %u", netStats.snapshotPacketsSent);
                    ImGui::Text("Snapshot packets received : %u", netStats.snapshotPacketsReceived);
                    ImGui::Text("Snapshot packets dropped  : %u", netStats.snapshotPacketsDropped);
                    ImGui::Text("Snapshot packets delayed  : %u", netStats.snapshotPacketsDelayed);

                    const uint32_t totalSnapshotAttempts =
                        netStats.snapshotPacketsSent + netStats.snapshotPacketsDropped;

                    const float measuredDropPercent =
                        totalSnapshotAttempts > 0
                        ? 100.0f * float(netStats.snapshotPacketsDropped) / float(totalSnapshotAttempts)
                        : 0.0f;

                    ImGui::Text("Measured drop rate        : %.1f%%", measuredDropPercent);

                    ImGui::Separator();

                    ImGui::Text("Global commands sent      : %u", netStats.globalCommandsSent);
                    ImGui::Text("Global commands received  : %u", netStats.globalCommandsReceived);
                    ImGui::Text("Spawn packets sent        : %u", netStats.spawnPacketsSent);
                    ImGui::Text("Spawn packets received    : %u", netStats.spawnPacketsReceived);
                    ImGui::Text("Reliable resends          : %u", netStats.reliableResends);

                    ImGui::Separator();

                    ImGui::Text("Delayed outgoing snapshots: %u", netStats.delayedOutgoingSnapshotPackets);
                    ImGui::Text("Delayed incoming snapshots: %u", netStats.delayedIncomingSnapshotPackets);
                }

                ImGui::Separator();

                ImGui::SeparatorText("Thread / Core Mapping");
                ImGui::Text("Available cores : %d", numCores);
                ImGui::Text("Render  thread  -> logical core %d", shared.renderCoreAssigned);
                ImGui::Text("Network thread  -> logical core %d", shared.netCoreAssigned);
                ImGui::Text("Sim     thread  -> logical core %d", shared.simCoreAssigned);
                ImGui::Separator();

                if (snap)
                    ImGui::Text("Snapshot tick   : %llu", (unsigned long long)snap->simTickNumber);
                else
                    ImGui::TextDisabled("No snapshot yet");

                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }

        renderer.EndImGuiFrame();

        if (snap)
        {
            RenderScene scene{};
            scene.camera = BuildCameraRenderData(renderCamera, renderer.getAspectRatio());
            scene.ambientLight = snap->ambientLight;
            scene.clearColor = snap->clearColor;
            scene.instances = snap->instances;
            scene.particles = snap->particles;
            scene.directionalLights = snap->directionalLights;
            scene.sparkLights = snap->sparkLights;
            renderer.render(scene);
        }

        renderLC.endFrame();
    }

    // Shutdown
    shared.appRunning.store(false, std::memory_order_relaxed);

    if (simThread.joinable()) simThread.join();
    if (netThread.joinable()) netThread.join();

    return 0;
}
