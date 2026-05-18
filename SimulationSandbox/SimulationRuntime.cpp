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
#include "FlockingSystem.h"
#include "GpuFlockingCompute.h"

#include "PeerConfig.h"
#include "WorldSnapshot.h"
#include "ThreadUtils.h"
#include "LoopController.h"
#include "RigidBody.h"
#include "ReplicaSmoothing.h"
#include "SandboxCameraUtils.h"
#include "SandboxSnapshot.h"

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
#include <sstream>
#include <limits>

#include "NetworkingSystem.h"
#include "Components.h" // OwnerComponent

struct RateCounter
{
    double windowStartSec = 0.0;
    uint32_t count = 0;

    float Add(uint32_t amount, double nowSec)
    {
        if (windowStartSec <= 0.0)
        {
            windowStartSec = nowSec;
            count = 0;
            return -1.0f;
        }

        count += amount;
        const double elapsed = nowSec - windowStartSec;
        if (elapsed < 0.5)
            return -1.0f;

        const float hz = elapsed > 0.0
            ? static_cast<float>(static_cast<double>(count) / elapsed)
            : 0.0f;

        windowStartSec = nowSec;
        count = 0;
        return hz;
    }
};

struct SimTimingStats
{
    float simLoopMsAvg = 0.0f;
    float simLoopMsMax = 0.0f;
    float scenarioMsAvg = 0.0f;
    float scenarioMsMax = 0.0f;
    float physicsMsAvg = 0.0f;
    float physicsMsMax = 0.0f;
    float collisionMsAvg = 0.0f;
    float collisionMsMax = 0.0f;
    float netStateMsAvg = 0.0f;
    float netStateMsMax = 0.0f;
    float renderSnapshotMsAvg = 0.0f;
    float renderSnapshotMsMax = 0.0f;
    float lockMsAvg = 0.0f;
    float lockMsMax = 0.0f;
    uint32_t fixedStepsLast = 0;
    uint32_t fixedStepsMaxAllowed = 0;
    float accumulatorBeforeMs = 0.0f;
    float accumulatorAfterMs = 0.0f;
    float accumulatorDroppedMs = 0.0f;
    float estimatedSleepWaitMs = 0.0f;
};

struct TimingSeries
{
    double totalMs = 0.0;
    double maxMs = 0.0;
    uint32_t samples = 0;

    void Add(double ms)
    {
        totalMs += ms;
        maxMs = std::max(maxMs, ms);
        ++samples;
    }

    float Avg() const
    {
        return samples > 0
            ? static_cast<float>(totalMs / static_cast<double>(samples))
            : 0.0f;
    }

    float Max() const
    {
        return static_cast<float>(maxMs);
    }
};

struct TimingAccumulator
{
    TimingSeries simLoop;
    TimingSeries scenario;
    TimingSeries physics;
    TimingSeries collision;
    TimingSeries netState;
    TimingSeries renderSnapshot;
    TimingSeries lock;
    double windowStartSec = 0.0;

    static double Ms(double startSec, double endSec)
    {
        return (endSec - startSec) * 1000.0;
    }

    bool PublishIfReady(double nowSec, SimTimingStats& out)
    {
        if (windowStartSec <= 0.0)
            windowStartSec = nowSec;

        if ((nowSec - windowStartSec) < 0.5)
            return false;

        out.simLoopMsAvg = simLoop.Avg();
        out.simLoopMsMax = simLoop.Max();
        out.scenarioMsAvg = scenario.Avg();
        out.scenarioMsMax = scenario.Max();
        out.physicsMsAvg = physics.Avg();
        out.physicsMsMax = physics.Max();
        out.collisionMsAvg = collision.Avg();
        out.collisionMsMax = collision.Max();
        out.netStateMsAvg = netState.Avg();
        out.netStateMsMax = netState.Max();
        out.renderSnapshotMsAvg = renderSnapshot.Avg();
        out.renderSnapshotMsMax = renderSnapshot.Max();
        out.lockMsAvg = lock.Avg();
        out.lockMsMax = lock.Max();

        *this = TimingAccumulator{};
        windowStartSec = nowSec;
        return true;
    }
};

struct RuntimeDebugCounts
{
    uint32_t localOwned = 0;
    uint32_t remoteOwned = 0;
    uint32_t staticObjects = 0;
    uint32_t animatedObjects = 0;
    uint32_t spawnedObjects = 0;
    uint32_t dynamicBodies = 0;
    uint32_t collisionCandidates = 0;
    uint32_t collisionBroadPhaseRejected = 0;
    uint32_t collisionSolidSolidCandidates = 0;
    uint32_t collisionSolidContainerCandidates = 0;
    uint32_t collisionContainerBroadPhaseRejected = 0;
    uint32_t collisionSpatialCells = 0;
    uint32_t collisionSolverVelocityIterations = 0;
    uint32_t collisionContacts = 0;
};

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
    std::atomic<int>   integratorType{ static_cast<int>(IntegratorType::SemiImplicitEuler) };

    // Scene switching: render thread writes, sim thread reads and acts
    std::atomic<int> requestedSceneIndex{ -1 };
    std::atomic<bool> requestedSceneReload{ false };
    std::atomic<uint32_t> sceneCameraResetGeneration{ 0 };
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
    std::atomic<float> measuredNetworkSendHz{ 0.0f };
    std::atomic<float> measuredNetworkRecvHz{ 0.0f };
    std::atomic<float> measuredSimHz{ 0.0f };
    std::atomic<float> measuredSimLoopHz{ 0.0f };
    std::mutex netStatsMutex;
    Net::NetworkStats netStats{};
    std::mutex activePeerIdsMutex;
    std::vector<int> activePeerIds;
    std::mutex peerDebugMutex;
    std::vector<Net::PeerDebugInfo> peerDebugInfo;
    std::mutex simTimingMutex;
    SimTimingStats simTiming{};
    std::atomic<uint32_t> fixedStepsLast{ 0 };
    std::atomic<uint32_t> fixedStepsMaxAllowed{ 12 };
    std::atomic<float> accumulatorBeforeMs{ 0.0f };
    std::atomic<float> accumulatorAfterMs{ 0.0f };
    std::atomic<float> accumulatorDroppedMs{ 0.0f };
    std::mutex flockingStatsMutex;
    FlockingStats flockingStats{};
    std::atomic<bool> flockingEnabled{ true };
    std::atomic<bool> flockingDebugEnabled{ false };
    std::atomic<bool> flockingBoundsEnabled{ true };
    std::atomic<bool> flockingUseUiSettings{ true };
    std::atomic<float> flockingMaxSpeed{ 8.0f };
    std::atomic<float> flockingMaxForce{ 15.0f };
    std::atomic<float> flockingPerceptionRadius{ 6.0f };
    std::atomic<float> flockingSeparationRadius{ 1.5f };
    std::atomic<float> flockingCohesionWeight{ 0.8f };
    std::atomic<float> flockingAlignmentWeight{ 1.0f };
    std::atomic<float> flockingSeparationWeight{ 1.5f };
    std::atomic<float> flockingAvoidanceWeight{ 2.5f };
    std::atomic<int> flockingDebugAgentCount{ 0 };
    std::atomic<int> flockingSearchMode{ static_cast<int>(FlockingNeighbourSearchMode::BruteForce) };
    std::mutex debugCountsMutex;
    RuntimeDebugCounts debugCounts{};
    // Core indices actually assigned (for UI display)
    int renderCoreAssigned = ThreadUtils::CORE_RENDER;
    int netSendCoreAssigned = ThreadUtils::CORE_NET_0;
    int netRecvCoreAssigned = ThreadUtils::CORE_NET_1;
    std::atomic<bool> netSendThreadRunning{ false };
    std::atomic<bool> netRecvThreadRunning{ false };
    int simCoreAssigned = ThreadUtils::CORE_SIM_0;
    std::mutex simWorkerCoresMutex;
    std::vector<int> simWorkerCores;
    std::atomic<int> simWorkerThreadCount{ 0 };

    // ---- Milestone 5: outgoing snapshot (sim -> net) ----
    std::mutex outSnapMutex;
    std::vector<Net::StateSnapshotItem> outOwnedItems;
    uint32_t outTick = 0;
    bool outDirty = false;

    // ---- Milestone 5: incoming replica snapshots (net -> sim) ----
    std::mutex inSnapMutex;
    std::unordered_map<uint32_t, ReplicaSnapshotRing> inReplicaHistory; // objectId -> received snapshot ring
    std::unordered_map<uint32_t, uint32_t> inReplicaLatestTick;
    std::atomic<uint32_t> staleReplicaPacketsIgnored{ 0 };

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
    std::atomic<float> replicaCorrectionStrength{ 1.0f };

    // Local network impairment tools (affects STATE_SNAPSHOT packets only)
    std::atomic<bool>  netImpairmentEnabled{ false };
    std::atomic<float> netLatencyMs{ 0.0f };
    std::atomic<float> netJitterMs{ 0.0f };
    std::atomic<float> netDropPercent{ 0.0f };
    std::atomic<float> snapshotSendHz{ 0.0f };

    // Snapshot buffer (sim -> render)
    SnapshotBuffer snapshotBuf;
};

// ============================================================================
static RuntimeDebugCounts BuildRuntimeDebugCounts(
    World& world,
    int localPeerId,
    const PhysicsSystem& physicsSystem,
    const CollisionSystem& collisionSystem)
{
    RuntimeDebugCounts counts{};
    world.forEach<OwnerComponent>([&](Entity e, OwnerComponent& owner)
        {
            if (owner.ownerId < 0)
            {
                if (world.getComponent<AnimatedPathComponent>(e))
                    ++counts.animatedObjects;
                else if (world.getComponent<PhysicsComponent>(e))
                    ++counts.staticObjects;
            }
            else if (owner.ownerId == localPeerId - 1)
            {
                ++counts.localOwned;
            }
            else
            {
                ++counts.remoteOwned;
            }

            if (world.getComponent<PhysicsComponent>(e) &&
                world.getComponent<NameComponent>(e) &&
                e >= 1000)
            {
                ++counts.spawnedObjects;
            }
        });

    counts.dynamicBodies = static_cast<uint32_t>(physicsSystem.DynamicBodyCount());
    counts.collisionCandidates = collisionSystem.LastCandidatePairs();
    counts.collisionBroadPhaseRejected = collisionSystem.LastBroadPhaseRejectedPairs();
    counts.collisionSolidSolidCandidates = collisionSystem.LastSolidSolidCandidatePairs();
    counts.collisionSolidContainerCandidates = collisionSystem.LastSolidContainerCandidatePairs();
    counts.collisionContainerBroadPhaseRejected = collisionSystem.LastContainerBroadPhaseRejectedPairs();
    counts.collisionSpatialCells = collisionSystem.LastSpatialCells();
    counts.collisionSolverVelocityIterations = collisionSystem.LastSolverVelocityIterations();
    counts.collisionContacts = collisionSystem.LastContactCount();
    return counts;
}

static void ApplyFlockingUiSettings(World& world, const SimSharedState& shared)
{
    if (!shared.flockingUseUiSettings.load(std::memory_order_relaxed))
        return;

    const float maxSpeed = shared.flockingMaxSpeed.load(std::memory_order_relaxed);
    const float maxForce = shared.flockingMaxForce.load(std::memory_order_relaxed);
    const float perceptionRadius = shared.flockingPerceptionRadius.load(std::memory_order_relaxed);
    const float separationRadius = shared.flockingSeparationRadius.load(std::memory_order_relaxed);
    const float cohesionWeight = shared.flockingCohesionWeight.load(std::memory_order_relaxed);
    const float alignmentWeight = shared.flockingAlignmentWeight.load(std::memory_order_relaxed);
    const float separationWeight = shared.flockingSeparationWeight.load(std::memory_order_relaxed);
    const float avoidanceWeight = shared.flockingAvoidanceWeight.load(std::memory_order_relaxed);

    world.forEach<FlockingComponent>([&](Entity, FlockingComponent& flock)
        {
            flock.maxSpeed = maxSpeed;
            flock.maxForce = maxForce;
            flock.perceptionRadius = perceptionRadius;
            flock.separationRadius = separationRadius;
            flock.cohesionWeight = cohesionWeight;
            flock.alignmentWeight = alignmentWeight;
            flock.separationWeight = separationWeight;
            flock.avoidanceWeight = avoidanceWeight;
        });
}

static const char* FlockingSearchModeName(FlockingNeighbourSearchMode mode)
{
    switch (mode)
    {
    case FlockingNeighbourSearchMode::UniformGrid: return "Uniform Grid";
    case FlockingNeighbourSearchMode::Octree: return "Octree";
    case FlockingNeighbourSearchMode::GpuComputeBruteForce: return "GPU Compute Brute Force";
    case FlockingNeighbourSearchMode::BruteForce:
    default: return "Brute Force";
    }
}

static void RefreshFlatbufferScenarioPeerFallbacks(
    ScenarioManager& scenarios,
    SimSharedState& shared,
    int localPeerId)
{
    std::vector<int> activePeerIds{ localPeerId };
    {
        std::lock_guard<std::mutex> lk(shared.activePeerIdsMutex);
        for (int peerId : shared.activePeerIds)
        {
            if (peerId >= 1 && peerId <= 4 &&
                std::find(activePeerIds.begin(), activePeerIds.end(), peerId) == activePeerIds.end())
            {
                activePeerIds.push_back(peerId);
            }
        }
    }

    for (int i = 0; i < scenarios.Count(); ++i)
    {
        if (auto* fb = dynamic_cast<Scenario_FlatbufferScene*>(scenarios.Get(i)))
        {
            fb->SetLocalPeerId(localPeerId);
            fb->SetConfiguredPeerIds(activePeerIds);
        }
    }
}

static std::shared_ptr<WorldSnapshot> CaptureSnapshotFromShared(
    World& world,
    uint64_t tickNumber,
    SimSharedState& shared)
{
    glm::vec4 clearColor;
    {
        std::lock_guard<std::mutex> lk(shared.clearColourMutex);
        clearColor = glm::vec4(
            shared.clearColour[0],
            shared.clearColour[1],
            shared.clearColour[2],
            shared.clearColour[3]);
    }

    return CaptureSnapshot(
        world,
        tickNumber,
        shared.displayMode.load(std::memory_order_relaxed),
        clearColor);
}

// ============================================================================
// Simulation thread entry point
// ============================================================================
static void SimulationThreadFunc(SimSharedState& shared, World& world,
    ScenarioManager& scenarios,
    PhysicsSystem& physicsSystem,
    CollisionSystem& collisionSystem,
    FlockingSystem& flockingSystem,
    int localPeerId)
{
    // Affinity + name
    const int numCores = ThreadUtils::LogicalCoreCount();
    const int assignedCore = (ThreadUtils::CORE_SIM_0 < numCores)
        ? ThreadUtils::CORE_SIM_0
        : (numCores - 1);
    ThreadUtils::PinCurrentThreadToCore(assignedCore);
    ThreadUtils::SetCurrentThreadName("Sim");
    shared.simCoreAssigned = assignedCore;
    flockingSystem.SetLocalPeerId(localPeerId);

    std::vector<int> simWorkerCores;
    for (int core = ThreadUtils::CORE_SIM_0; core < numCores; ++core)
        simWorkerCores.push_back(core);
    if (simWorkerCores.empty())
        simWorkerCores.push_back(assignedCore);

    physicsSystem.SetWorkerCores(simWorkerCores);
    physicsSystem.SetWorkerCount(static_cast<int>(simWorkerCores.size()));
    shared.simWorkerThreadCount.store(
        std::max(0, static_cast<int>(simWorkerCores.size()) - 1),
        std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(shared.simWorkerCoresMutex);
        shared.simWorkerCores = simWorkerCores;
    }

    LoopController lc(shared.targetSimHz.load(std::memory_order_relaxed));
    uint64_t tickNumber = 0;
    float    accumulator = 0.0f;
    float    renderSnapshotAccum = 0.0f;
    float    netStateGenerationAccum = 0.0f;
    RateCounter simTickRate;
    TimingAccumulator timing;
    constexpr int MAX_FIXED_SUBSTEPS_PER_FRAME = 12;
    shared.fixedStepsMaxAllowed.store(MAX_FIXED_SUBSTEPS_PER_FRAME, std::memory_order_relaxed);
    const float maxDt = 0.05f;
    double lastDebugCountsSec = 0.0;

    while (shared.appRunning.load(std::memory_order_relaxed))
    {
        const double simLoopStartSec = NowSeconds();
        // Live-apply target Hz changes
        lc.setTargetHz(shared.targetSimHz.load(std::memory_order_relaxed));

        float dt = lc.beginFrame();
        shared.measuredSimLoopHz.store(lc.getMeasuredHz(), std::memory_order_relaxed);

        // Clamp to avoid runaway catch-up
        dt = std::min(dt, maxDt);
        uint32_t simTicksThisLoop = 0;
        float accumulatorBeforeStep = accumulator;
        float accumulatorDroppedThisLoop = 0.0f;

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
            shared.requestedSceneReload.store(true, std::memory_order_release);
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
                const bool forceReload = shared.requestedSceneReload.exchange(false, std::memory_order_acq_rel);
                if (forceReload || desired != scenarios.CurrentIndex())
                {
                    const ULONGLONG t = GetTickCount64();
                    std::cout << "[T] SIM applying scene idx=" << desired
                        << " t=" << t << "ms\n";
                    RefreshFlatbufferScenarioPeerFallbacks(scenarios, shared, localPeerId);
                    scenarios.SwitchTo(world, desired);
                    physicsSystem.InitializePhysicsBodies(world);
                    shared.sceneCameraResetGeneration.store(
                        shared.sceneGeneration.load(std::memory_order_acquire),
                        std::memory_order_release);
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
                        shared.inReplicaLatestTick.clear();
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
            const float correctionStrength = shared.replicaCorrectionStrength.load(std::memory_order_relaxed);
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
                auto* vel = world.getComponent<VelocityComponent>(e);
                auto* flock = world.getComponent<FlockingComponent>(e);
                if (!tr) continue;
                if (!phys && !flock) continue;

                // Don't override authoritative bodies
                auto* owner = world.getComponent<OwnerComponent>(e);
                const bool isAnimated = world.getComponent<AnimatedPathComponent>(e) != nullptr;

                // Never apply remote snapshots onto objects owned by this local peer.
                if (owner && owner->ownerId == localPeerId - 1)
                    continue;

                // Static/local-all objects are usually local, but animated
                // kinematic bodies are synchronised from peer 1 so replicas
                // collide visually against the same moving surface as the owner.
                if (owner && owner->ownerId < 0 && !isAnimated)
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
                if (!smoothingEnabled)
                {
                    if (phys)
                    {
                        phys->body.SetPosition(target.pos);
                        phys->body.SetOrientation(target.rot);
                        phys->body.SetLinearVelocity(target.linVel);
                        phys->body.SetAngularVelocity(target.angVel);
                    }
                    if (vel)
                    {
                        vel->linearVelocity = target.linVel;
                        vel->angularVelocity = target.angVel;
                    }
                    tr->position = target.pos;
                    tr->rotation = glm::eulerAngles(target.rot);
                    tr->orientation = target.rot;
                    continue;
                }

                const glm::vec3 currentPos = phys ? phys->body.Position() : tr->position;
                const glm::quat currentRot = phys ? phys->body.Orientation() : tr->orientation;
                const glm::vec3 currentLinVel = phys ? phys->body.LinearVelocity() : (vel ? vel->linearVelocity : glm::vec3{ 0.0f });
                const glm::vec3 currentAngVel = phys ? phys->body.AngularVelocity() : (vel ? vel->angularVelocity : glm::vec3{ 0.0f });

                const float posError = glm::length(target.pos - currentPos);
                const bool largeError = posError > correctionThreshold;

                float spring = (largeError ? 7.0f : 16.0f) * std::max(0.0f, correctionStrength);
                if (usedExtrapolation)
                    spring *= 0.75f;

                const float blend = isAnimated ? 1.0f : glm::clamp(spring * smoothingDt, 0.0f, 1.0f);

                const glm::vec3 smoothPos = glm::mix(currentPos, target.pos, blend);
                const glm::quat smoothRot = glm::normalize(glm::slerp(currentRot, target.rot, blend));
                const glm::vec3 smoothLinVel = glm::mix(currentLinVel, target.linVel, blend);
                const glm::vec3 smoothAngVel = glm::mix(currentAngVel, target.angVel, blend);

                if (phys)
                {
                    phys->body.SetPosition(smoothPos);
                    phys->body.SetOrientation(smoothRot);
                    phys->body.SetLinearVelocity(smoothLinVel);
                    phys->body.SetAngularVelocity(smoothAngVel);
                }
                if (vel)
                {
                    vel->linearVelocity = smoothLinVel;
                    vel->angularVelocity = smoothAngVel;
                }

                tr->position = smoothPos;
                tr->rotation = glm::eulerAngles(smoothRot);
                tr->orientation = smoothRot;
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
                        shared.inReplicaLatestTick.clear();
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

                if (!incomingSpawns.empty())
                    physicsSystem.InitializePhysicsBodies(world);
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

                int substepsThisFrame = 0;
                accumulatorBeforeStep = accumulator;

                while (accumulator >= fdt && substepsThisFrame < MAX_FIXED_SUBSTEPS_PER_FRAME)
                {
                    size_t pendingSpawnsBeforeUpdate = 0;
                    if (auto* fb = dynamic_cast<Scenario_FlatbufferScene*>(s))
                        pendingSpawnsBeforeUpdate = fb->PendingSpawnCount();

                    const double scenarioStartSec = NowSeconds();
                    s->Update(world, fdt, currentSceneGeneration);
                    const double flockingStartSec = NowSeconds();

                    if (auto* fb = dynamic_cast<Scenario_FlatbufferScene*>(s))
                    {
                        if (fb->PendingSpawnCount() != pendingSpawnsBeforeUpdate)
                            physicsSystem.InitializePhysicsBodies(world);
                    }

                    flockingSystem.SetEnabled(shared.flockingEnabled.load(std::memory_order_relaxed));
                    flockingSystem.SetDebugEnabled(shared.flockingDebugEnabled.load(std::memory_order_relaxed));
                    flockingSystem.SetBoundsEnabled(shared.flockingBoundsEnabled.load(std::memory_order_relaxed));
                    flockingSystem.SetNeighbourSearchMode(static_cast<FlockingNeighbourSearchMode>(
                        shared.flockingSearchMode.load(std::memory_order_relaxed)));
                    ApplyFlockingUiSettings(world, shared);
                    flockingSystem.Update(world, fdt);
                    {
                        std::lock_guard<std::mutex> lk(shared.flockingStatsMutex);
                        shared.flockingStats = flockingSystem.GetStats();
                    }
                    shared.flockingDebugAgentCount.store(
                        static_cast<int>(flockingSystem.GetDebugAgents().size()),
                        std::memory_order_relaxed);
                    const double physicsStartSec = NowSeconds();
                    physicsSystem.Update(world, fdt);
                    const double collisionStartSec = NowSeconds();
                    collisionSystem.Update(world);
                    const double collisionEndSec = NowSeconds();

                    timing.scenario.Add(TimingAccumulator::Ms(scenarioStartSec, flockingStartSec));
                    timing.physics.Add(TimingAccumulator::Ms(physicsStartSec, collisionStartSec));
            timing.collision.Add(TimingAccumulator::Ms(collisionStartSec, collisionEndSec));

                    accumulator -= fdt;
                    ++tickNumber;
                    ++substepsThisFrame;
                    ++simTicksThisLoop;
                }

                if (accumulator >= fdt)
                {
                    accumulatorDroppedThisLoop = accumulator;
                    accumulator = 0.0f;
                }
            }
            else
            {
                if (running || doStep)
                {
                    const float stepDt = running ? dt : fdt;
                    size_t pendingSpawnsBeforeUpdate = 0;
                    if (auto* fb = dynamic_cast<Scenario_FlatbufferScene*>(s))
                        pendingSpawnsBeforeUpdate = fb->PendingSpawnCount();

                    const double scenarioStartSec = NowSeconds();
                    s->Update(world, stepDt, currentSceneGeneration);
                    const double flockingStartSec = NowSeconds();

                    if (auto* fb = dynamic_cast<Scenario_FlatbufferScene*>(s))
                    {
                        if (fb->PendingSpawnCount() != pendingSpawnsBeforeUpdate)
                            physicsSystem.InitializePhysicsBodies(world);
                    }

                    flockingSystem.SetEnabled(shared.flockingEnabled.load(std::memory_order_relaxed));
                    flockingSystem.SetDebugEnabled(shared.flockingDebugEnabled.load(std::memory_order_relaxed));
                    flockingSystem.SetBoundsEnabled(shared.flockingBoundsEnabled.load(std::memory_order_relaxed));
                    flockingSystem.SetNeighbourSearchMode(static_cast<FlockingNeighbourSearchMode>(
                        shared.flockingSearchMode.load(std::memory_order_relaxed)));
                    ApplyFlockingUiSettings(world, shared);
                    flockingSystem.Update(world, stepDt);
                    {
                        std::lock_guard<std::mutex> lk(shared.flockingStatsMutex);
                        shared.flockingStats = flockingSystem.GetStats();
                    }
                    shared.flockingDebugAgentCount.store(
                        static_cast<int>(flockingSystem.GetDebugAgents().size()),
                        std::memory_order_relaxed);
                    const double physicsStartSec = NowSeconds();
                    physicsSystem.Update(world, stepDt);
                    const double collisionStartSec = NowSeconds();
                    collisionSystem.Update(world);
                    const double collisionEndSec = NowSeconds();

                    timing.scenario.Add(TimingAccumulator::Ms(scenarioStartSec, flockingStartSec));
                    timing.physics.Add(TimingAccumulator::Ms(physicsStartSec, collisionStartSec));
                    timing.collision.Add(TimingAccumulator::Ms(collisionStartSec, collisionEndSec));

                    ++tickNumber;
                    ++simTicksThisLoop;
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

        const double nowSecForRate = NowSeconds();
        shared.fixedStepsLast.store(simTicksThisLoop, std::memory_order_relaxed);
        shared.accumulatorBeforeMs.store(accumulatorBeforeStep * 1000.0f, std::memory_order_relaxed);
        shared.accumulatorAfterMs.store(accumulator * 1000.0f, std::memory_order_relaxed);
        shared.accumulatorDroppedMs.store(accumulatorDroppedThisLoop * 1000.0f, std::memory_order_relaxed);

        const float tickHz = simTickRate.Add(simTicksThisLoop, nowSecForRate);
        if (tickHz >= 0.0f)
            shared.measuredSimHz.store(tickHz, std::memory_order_relaxed);

        // ----- Milestone 5: build outgoing STATE_SNAPSHOT (owned dynamics) -----
        const float snapshotHzForGeneration = shared.snapshotSendHz.load(std::memory_order_relaxed);
        const float snapshotGenerationInterval =
            (snapshotHzForGeneration > 0.0f) ? (1.0f / snapshotHzForGeneration) : 0.0f;
        netStateGenerationAccum += dt;
        const bool generateNetworkState =
            simTicksThisLoop > 0 &&
            (snapshotGenerationInterval <= 0.0f || netStateGenerationAccum >= snapshotGenerationInterval);

        if (generateNetworkState)
        {
            netStateGenerationAccum = 0.0f;
            const double netStateStartSec = NowSeconds();
            std::vector<Net::StateSnapshotItem> items;
            items.reserve(4096);

            world.forEach<PhysicsComponent>([&](Entity e, PhysicsComponent& phys)
                {
                auto* owner = world.getComponent<OwnerComponent>(e);
                const bool isAnimated = world.getComponent<AnimatedPathComponent>(e) != nullptr;

                    // Only simulated owned objects should be sent by this peer.
                    // Animated objects are authored as local/all-peer, but peer 1
                    // publishes their kinematic transform so remote simulated bodies
                    // and animated collision surfaces share one timing source.
                    if (!owner)
                        return;

                    const bool shouldSendOwnedDynamic = owner->ownerId == localPeerId - 1;
                    const bool shouldSendAnimated = isAnimated && owner->ownerId < 0 && localPeerId == 1;
                    if (!shouldSendOwnedDynamic && !shouldSendAnimated)
                        return;

                    if (!phys.body.IsDynamic() && !shouldSendAnimated)
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

            world.forEach<FlockingComponent>([&](Entity e, FlockingComponent& flock)
                {
                    if (!flock.enabled)
                        return;

                    auto* owner = world.getComponent<OwnerComponent>(e);
                    auto* tr = world.getComponent<TransformComponent>(e);
                    auto* vel = world.getComponent<VelocityComponent>(e);
                    if (!owner || !tr || !vel)
                        return;

                    if (owner->ownerId != localPeerId - 1)
                        return;

                    // Flocking boids are intentionally not PhysicsComponent bodies,
                    // but their owner still needs to replicate transform/velocity.
                    Net::StateSnapshotItem it{};
                    it.objectId = (uint32_t)e;

                    const glm::quat q = glm::normalize(tr->orientation);
                    it.pos = { tr->position.x, tr->position.y, tr->position.z };
                    it.rot = { q.x, q.y, q.z, q.w };
                    it.linVel = { vel->linearVelocity.x, vel->linearVelocity.y, vel->linearVelocity.z };
                    it.angVel = { vel->angularVelocity.x, vel->angularVelocity.y, vel->angularVelocity.z };

                    items.push_back(it);
                });

            const double lockStartSec = NowSeconds();
            {
                std::lock_guard<std::mutex> lk(shared.outSnapMutex);
                shared.outOwnedItems = std::move(items);
                shared.outTick = (uint32_t)tickNumber;
                shared.outDirty = true;
            }
            const double lockEndSec = NowSeconds();

            timing.netState.Add(TimingAccumulator::Ms(netStateStartSec, lockStartSec));
            timing.lock.Add(TimingAccumulator::Ms(lockStartSec, lockEndSec));
        }

        const float renderHz = shared.targetRenderHz.load(std::memory_order_relaxed);
        const float renderSnapshotInterval = (renderHz > 0.0f) ? (1.0f / renderHz) : 0.0f;
        renderSnapshotAccum += dt;
        if (renderSnapshotInterval <= 0.0f || renderSnapshotAccum >= renderSnapshotInterval)
        {
            renderSnapshotAccum = 0.0f;
            const double renderSnapshotStartSec = NowSeconds();
            shared.snapshotBuf.publish(CaptureSnapshotFromShared(world, tickNumber, shared));
            const double renderSnapshotEndSec = NowSeconds();
            timing.renderSnapshot.Add(TimingAccumulator::Ms(renderSnapshotStartSec, renderSnapshotEndSec));
        }

        timing.simLoop.Add(TimingAccumulator::Ms(simLoopStartSec, NowSeconds()));

        SimTimingStats publishedTiming{};
        if (timing.PublishIfReady(NowSeconds(), publishedTiming))
        {
            publishedTiming.fixedStepsLast = shared.fixedStepsLast.load(std::memory_order_relaxed);
            publishedTiming.fixedStepsMaxAllowed = shared.fixedStepsMaxAllowed.load(std::memory_order_relaxed);
            publishedTiming.accumulatorBeforeMs = shared.accumulatorBeforeMs.load(std::memory_order_relaxed);
            publishedTiming.accumulatorAfterMs = shared.accumulatorAfterMs.load(std::memory_order_relaxed);
            publishedTiming.accumulatorDroppedMs = shared.accumulatorDroppedMs.load(std::memory_order_relaxed);
            const float targetSimHz = shared.targetSimHz.load(std::memory_order_relaxed);
            const float targetLoopMs = targetSimHz > 0.0f ? 1000.0f / targetSimHz : 0.0f;
            publishedTiming.estimatedSleepWaitMs = std::max(0.0f, targetLoopMs - publishedTiming.simLoopMsAvg);
            std::lock_guard<std::mutex> lk(shared.simTimingMutex);
            shared.simTiming = publishedTiming;
        }

        if ((nowSecForRate - lastDebugCountsSec) >= 0.1)
        {
            lastDebugCountsSec = nowSecForRate;
            RuntimeDebugCounts counts =
                BuildRuntimeDebugCounts(world, localPeerId, physicsSystem, collisionSystem);
            std::lock_guard<std::mutex> lk(shared.debugCountsMutex);
            shared.debugCounts = counts;
        }

        lc.endFrame();
    }
}

#include "SandboxNetworkThreads.inl"
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
    FlockingSystem   flockingSystem;
    flockingSystem.SetLocalPeerId(localPeerId);
    GpuFlockingCompute gpuFlocking;
    const bool gpuFlockingReady = gpuFlocking.Initialise(
        renderer.GetDevice(),
        renderer.GetPhysicalDevice(),
        renderer.GetGraphicsQueue(),
        renderer.GetGraphicsQueueFamily(),
        &renderer.GetQueueMutex());
    flockingSystem.SetGpuCompute(&gpuFlocking);
    ScenarioManager  scenarios;

    scenarios.Add(std::make_unique<Scenario_PrimitiveScene>());
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/flatbufferFlockingDemo.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/sequentialSpawnerContainer.bin"));
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
        {
            fb->SetLocalPeerId(localPeerId);
            fb->SetConfiguredPeerIds(std::vector<int>{ localPeerId });
        }
    }

    if (scenarios.Count() > 0)
        scenarios.SwitchTo(world, 0);
    physicsSystem.InitializePhysicsBodies(world);

    SimSharedState shared;
    shared.targetRenderHz.store(cfg.render_hz, std::memory_order_relaxed);
    shared.targetNetworkHz.store(cfg.network_hz, std::memory_order_relaxed);
    shared.targetSimHz.store(cfg.simulation_hz, std::memory_order_relaxed);
    if (cfg.simulation_hz > 0.0f)
        shared.fixedDt.store(1.0f / cfg.simulation_hz, std::memory_order_relaxed);

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
                    renderCamera.projection = cam->projection;
                    renderCamera.fov = cam->fov;
                    renderCamera.orthoSize = cam->orthoSize;
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
    shared.snapshotBuf.publish(CaptureSnapshotFromShared(world, 0, shared));

    // Start background threads
    std::thread simThread(
        SimulationThreadFunc,
        std::ref(shared),
        std::ref(world),
        std::ref(scenarios),
        std::ref(physicsSystem),
        std::ref(collisionSystem),
        std::ref(flockingSystem),
        localPeerId
    );

    NetworkRuntime networkRuntime;
    const bool networkReady = networkRuntime.net.Init(cfg);
    std::thread netReceiveThread;
    std::thread netSendThread;
    if (networkReady)
    {
        netReceiveThread = std::thread(
            NetworkReceiveThreadFunc,
            std::ref(shared),
            std::cref(cfg),
            std::ref(networkRuntime));

        netSendThread = std::thread(
            NetworkSendThreadFunc,
            std::ref(shared),
            std::cref(cfg),
            std::ref(networkRuntime));
    }

    // Render / UI loop state
    LoopController renderLC(shared.targetRenderHz.load(std::memory_order_relaxed));
    Entity activeSceneCamera = INVALID_ENTITY;
    auto sceneCameraOptions = CollectSceneCameras(world);
    if (!sceneCameraOptions.empty())
    {
        activeSceneCamera = sceneCameraOptions.front().entity;
        ApplySceneCamera(world, activeSceneCamera, renderCamera);
    }
    uint32_t appliedSceneCameraGeneration =
        shared.sceneCameraResetGeneration.load(std::memory_order_acquire);
    const float maxFrameDt = 0.25f;

    float displayRenderHz = 0.0f;
    float displayNetHz = 0.0f;
    float displaySimHz = 0.0f;

    float uiRenderHz = cfg.render_hz;
    float uiNetHz = cfg.network_hz;
    float uiSimHz = cfg.simulation_hz;
    float uiSnapshotHz = shared.snapshotSendHz.load(std::memory_order_relaxed);

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
            const float yawDelta =
                (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS ? lookSpeed * dt : 0.0f) +
                (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS ? -lookSpeed * dt : 0.0f);
            const float pitchDelta =
                (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS ? lookSpeed * dt : 0.0f) +
                (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS ? -lookSpeed * dt : 0.0f);

            if (renderCamera.useOrientation)
            {
                if (yawDelta != 0.0f)
                {
                    renderCamera.orientation =
                        glm::normalize(glm::angleAxis(yawDelta, glm::vec3(0, 1, 0)) * renderCamera.orientation);
                }
                if (pitchDelta != 0.0f)
                {
                    const glm::mat3 basis = glm::mat3_cast(renderCamera.orientation);
                    const glm::vec3 right = glm::normalize(basis * glm::vec3(1, 0, 0));
                    const float signedPitchDelta = pitchDelta * -renderCamera.localForwardZ;
                    renderCamera.orientation =
                        glm::normalize(glm::angleAxis(signedPitchDelta, right) * renderCamera.orientation);
                }
                if (yawDelta != 0.0f || pitchDelta != 0.0f)
                    renderCamera.rotation = glm::eulerAngles(renderCamera.orientation);
            }
            else
            {
                renderCamera.rotation.y += yawDelta;
                renderCamera.rotation.x += pitchDelta;

                const float pitchLimit = glm::radians(89.0f);
                renderCamera.rotation.x = glm::clamp(renderCamera.rotation.x, -pitchLimit, pitchLimit);
            }

            const float moveSpeed = 10.0f;
            glm::vec3 moveLocal(0.0f);
            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) moveLocal.z += 1.0f;
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) moveLocal.z -= 1.0f;
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) moveLocal.x += 1.0f;
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) moveLocal.x -= 1.0f;
            if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) moveLocal.y -= 1.0f;
            if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) moveLocal.y += 1.0f;

            if (moveLocal.x != 0.0f || moveLocal.y != 0.0f || moveLocal.z != 0.0f)
            {
                moveLocal = glm::normalize(moveLocal);
                glm::vec3 forward{};
                glm::vec3 right{};
                if (renderCamera.useOrientation)
                {
                    const glm::mat3 basis = glm::mat3_cast(renderCamera.orientation);
                    forward = glm::normalize(basis * glm::vec3(0, 0, renderCamera.localForwardZ));
                    right = glm::normalize(glm::cross(forward, glm::vec3(0, 1, 0)));
                }
                else
                {
                    float yaw = renderCamera.rotation.y;
                    forward = glm::vec3(std::sin(yaw), 0.0f, std::cos(yaw));
                    right = glm::normalize(glm::cross(forward, glm::vec3(0, 1, 0)));
                }
                glm::vec3 up(0.0f, 1.0f, 0.0f);
                renderCamera.position += (right * moveLocal.x + up * moveLocal.y + forward * moveLocal.z) * (moveSpeed * dt);
            }
        }

        // Snapshot: consume latest from sim thread
        std::shared_ptr<WorldSnapshot> snap = shared.snapshotBuf.consume();

        const uint32_t cameraResetGeneration =
            shared.sceneCameraResetGeneration.load(std::memory_order_acquire);
        if (cameraResetGeneration != appliedSceneCameraGeneration)
        {
            activeSceneCamera = INVALID_ENTITY;
            appliedSceneCameraGeneration = cameraResetGeneration;
        }

        if (activeSceneCamera == INVALID_ENTITY)
        {
            sceneCameraOptions = CollectSceneCameras(world);
            if (!sceneCameraOptions.empty())
            {
                activeSceneCamera = sceneCameraOptions.front().entity;
                ApplySceneCamera(world, activeSceneCamera, renderCamera);
            }
        }

        // ImGui
        renderer.BeginImGuiFrame();

        displayRenderHz = shared.measuredRenderHz.load(std::memory_order_relaxed);
        displayNetHz = shared.measuredNetworkHz.load(std::memory_order_relaxed);
        displaySimHz = shared.measuredSimHz.load(std::memory_order_relaxed);

        // The coursework demo UI below replaces the old menu bar controls.
        if (false && ImGui::BeginMainMenuBar())
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
                sceneCameraOptions = CollectSceneCameras(world);
                if (!sceneCameraOptions.empty())
                {
                    ImGui::SeparatorText("Scene Cameras (Local)");
                    for (const auto& camOption : sceneCameraOptions)
                    {
                        const bool selected = (activeSceneCamera == camOption.entity);
                        std::string label = camOption.name;
                        label += camOption.projection == CameraComponent::Projection::Orthographic
                            ? " [Orthographic]"
                            : " [Perspective]";

                        if (ImGui::MenuItem(label.c_str(), nullptr, selected))
                        {
                            if (ApplySceneCamera(world, camOption.entity, renderCamera))
                                activeSceneCamera = camOption.entity;
                        }
                    }
                    ImGui::Separator();
                }
                else
                {
                    ImGui::TextDisabled("No FlatBuffer cameras in this scene");
                    ImGui::Separator();
                }

                ImGui::DragFloat3("Position", &renderCamera.position.x, 0.1f);
                if (ImGui::DragFloat3("Rotation (rad)", &renderCamera.rotation.x, 0.02f))
                    renderCamera.useOrientation = false;
                if (renderCamera.projection == CameraComponent::Projection::Orthographic)
                    ImGui::DragFloat("Ortho Size", &renderCamera.orthoSize, 0.25f, 0.1f, 1000.0f);
                else
                    ImGui::DragFloat("FOV", &renderCamera.fov, 0.5f, 10.0f, 170.0f);
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
                                shared.inReplicaLatestTick.clear();
                            }

                            {
                                std::lock_guard<std::mutex> lk(shared.inSpawnMutex);
                                shared.inSpawnEvents.clear();
                            }

                            // Start a short transition pause.
                            // The scene can load locally, but sim/spawners/snapshots should not run yet.
                            shared.sceneTransitionActive.store(true, std::memory_order_release);
                            shared.sceneTransitionRemainingSec.store(0.25f, std::memory_order_release);
                            std::cout << "[TRANSITION BEGIN UI] delay="
                                << shared.sceneTransitionRemainingSec.load(std::memory_order_acquire)

                                << "\n";
                            // Apply locally immediately.
                            shared.requestedSceneIndex.store(si, std::memory_order_release);
                            activeSceneCamera = INVALID_ENTITY;

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
                if (ImGui::SliderFloat("Sim Hz", &uiSimHz, 0.0f, 2000.0f, "%.0f"))
                {
                    shared.targetSimHz.store(uiSimHz, std::memory_order_relaxed);
                    if (uiSimHz > 0.0f)
                        shared.fixedDt.store(1.0f / uiSimHz, std::memory_order_relaxed);
                }
                if (ImGui::SliderFloat("Snapshot Send Hz", &uiSnapshotHz, 0.0f, 2000.0f, "%.0f"))
                {
                    const float minSnapshotHz = std::max(1.0f, shared.targetNetworkHz.load(std::memory_order_relaxed));
                    if (uiSnapshotHz > 0.0f && uiSnapshotHz < minSnapshotHz)
                        uiSnapshotHz = minSnapshotHz;
                    shared.snapshotSendHz.store(uiSnapshotHz, std::memory_order_relaxed);
                }
                ImGui::Text("Snapshot mode: %s",
                    uiSnapshotHz <= 0.0f
                    ? "every fresh simulation tick"
                    : "rate limited, minimum network tick rate");

                ImGui::TextDisabled("(0 = uncapped; snapshots at 0 = every fresh sim tick)");
                ImGui::Separator();

                ImGui::SeparatorText("Measured Frequencies");
                ImGui::Text("Render  : %6.1f Hz", displayRenderHz);
                ImGui::Text("Network : %6.1f Hz", displayNetHz);
                ImGui::Text("Net send: %6.1f Hz", shared.measuredNetworkSendHz.load(std::memory_order_relaxed));
                ImGui::Text("Net recv: %6.1f Hz", shared.measuredNetworkRecvHz.load(std::memory_order_relaxed));
                ImGui::Text("Sim     : %6.1f Hz", displaySimHz);
                ImGui::Text("Sim loop: %6.1f Hz", shared.measuredSimLoopHz.load(std::memory_order_relaxed));
                ImGui::Separator();

                ImGui::SeparatorText("Simulation Tick Timings");
                {
                    SimTimingStats timingStats{};
                    {
                        std::lock_guard<std::mutex> lk(shared.simTimingMutex);
                        timingStats = shared.simTiming;
                    }

                    ImGui::Text("Scenario/spawners  avg %.3f ms  max %.3f ms",
                        timingStats.scenarioMsAvg, timingStats.scenarioMsMax);
                    ImGui::Text("Physics update     avg %.3f ms  max %.3f ms",
                        timingStats.physicsMsAvg, timingStats.physicsMsMax);
                    ImGui::Text("Collision update   avg %.3f ms  max %.3f ms",
                        timingStats.collisionMsAvg, timingStats.collisionMsMax);
                    ImGui::Text("Network state gen  avg %.3f ms  max %.3f ms",
                        timingStats.netStateMsAvg, timingStats.netStateMsMax);
                    ImGui::Text("Render snapshot    avg %.3f ms  max %.3f ms",
                        timingStats.renderSnapshotMsAvg, timingStats.renderSnapshotMsMax);
                    ImGui::Text("Shared lock wait   avg %.3f ms  max %.3f ms",
                        timingStats.lockMsAvg, timingStats.lockMsMax);
                    ImGui::Text("Outer sim loop     avg %.3f ms  max %.3f ms",
                        timingStats.simLoopMsAvg, timingStats.simLoopMsMax);
                }
                ImGui::Separator();

                ImGui::SeparatorText("Flocking");
                {
                    bool flockingEnabled = shared.flockingEnabled.load(std::memory_order_relaxed);
                    if (ImGui::Checkbox("Enable Flocking", &flockingEnabled))
                        shared.flockingEnabled.store(flockingEnabled, std::memory_order_relaxed);

                    bool boundsEnabled = shared.flockingBoundsEnabled.load(std::memory_order_relaxed);
                    if (ImGui::Checkbox("Flocking Bounds", &boundsEnabled))
                        shared.flockingBoundsEnabled.store(boundsEnabled, std::memory_order_relaxed);

                    bool debugEnabled = shared.flockingDebugEnabled.load(std::memory_order_relaxed);
                    if (ImGui::Checkbox("Flocking Debug Data", &debugEnabled))
                        shared.flockingDebugEnabled.store(debugEnabled, std::memory_order_relaxed);

                    bool useUiSettings = shared.flockingUseUiSettings.load(std::memory_order_relaxed);
                    if (ImGui::Checkbox("Apply UI Settings To Boids", &useUiSettings))
                        shared.flockingUseUiSettings.store(useUiSettings, std::memory_order_relaxed);

                    int searchMode = shared.flockingSearchMode.load(std::memory_order_relaxed);
                    const char* searchModeLabels[] = { "Brute Force", "Uniform Grid", "Octree", "GPU Compute Brute Force" };
                    if (ImGui::Combo("Neighbour Search", &searchMode, searchModeLabels, IM_ARRAYSIZE(searchModeLabels)))
                        shared.flockingSearchMode.store(searchMode, std::memory_order_relaxed);

                    float maxSpeed = shared.flockingMaxSpeed.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Max Speed", &maxSpeed, 0.5f, 30.0f, "%.1f"))
                        shared.flockingMaxSpeed.store(maxSpeed, std::memory_order_relaxed);

                    float maxForce = shared.flockingMaxForce.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Max Force", &maxForce, 0.5f, 60.0f, "%.1f"))
                        shared.flockingMaxForce.store(maxForce, std::memory_order_relaxed);

                    float perceptionRadius = shared.flockingPerceptionRadius.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Perception Radius", &perceptionRadius, 0.5f, 20.0f, "%.1f"))
                    {
                        shared.flockingPerceptionRadius.store(perceptionRadius, std::memory_order_relaxed);
                        float separationRadius = shared.flockingSeparationRadius.load(std::memory_order_relaxed);
                        if (separationRadius > perceptionRadius)
                            shared.flockingSeparationRadius.store(perceptionRadius, std::memory_order_relaxed);
                    }

                    float separationRadius = shared.flockingSeparationRadius.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Separation Radius", &separationRadius, 0.1f, 10.0f, "%.1f"))
                    {
                        separationRadius = std::min(separationRadius, shared.flockingPerceptionRadius.load(std::memory_order_relaxed));
                        shared.flockingSeparationRadius.store(separationRadius, std::memory_order_relaxed);
                    }

                    ImGui::SeparatorText("Algorithm Weights");

                    float avoidanceWeight = shared.flockingAvoidanceWeight.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Avoidance Weight", &avoidanceWeight, 0.0f, 8.0f, "%.2f"))
                        shared.flockingAvoidanceWeight.store(avoidanceWeight, std::memory_order_relaxed);

                    float separationWeight = shared.flockingSeparationWeight.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Separation Weight", &separationWeight, 0.0f, 8.0f, "%.2f"))
                        shared.flockingSeparationWeight.store(separationWeight, std::memory_order_relaxed);

                    float alignmentWeight = shared.flockingAlignmentWeight.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Alignment Weight", &alignmentWeight, 0.0f, 8.0f, "%.2f"))
                        shared.flockingAlignmentWeight.store(alignmentWeight, std::memory_order_relaxed);

                    float cohesionWeight = shared.flockingCohesionWeight.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Cohesion Weight", &cohesionWeight, 0.0f, 8.0f, "%.2f"))
                        shared.flockingCohesionWeight.store(cohesionWeight, std::memory_order_relaxed);

                    ImGui::SeparatorText("Visualisation Stats");

                    FlockingStats flockStats{};
                    {
                        std::lock_guard<std::mutex> lk(shared.flockingStatsMutex);
                        flockStats = shared.flockingStats;
                    }
                    ImGui::Text("Agents           : %d", flockStats.agentCount);
                    ImGui::Text("Locally updated  : %d", flockStats.ownedAgentCount);
                    ImGui::Text("Search mode      : %s", FlockingSearchModeName(flockStats.searchMode));
                    ImGui::Text("Debug agents     : %d", shared.flockingDebugAgentCount.load(std::memory_order_relaxed));
                    ImGui::Text("Neighbour checks : %d", flockStats.neighbourChecks);
                    ImGui::Text("Spatial candidates: %d", flockStats.spatialCandidateChecks);
                    ImGui::Text("Neighbours found : %d", flockStats.neighboursFound);
                    ImGui::Text("Avg neighbours   : %.2f",
                        flockStats.ownedAgentCount > 0
                        ? static_cast<float>(flockStats.neighboursFound) / static_cast<float>(flockStats.ownedAgentCount)
                        : 0.0f);
                    ImGui::Text("Avoidance checks : %d", flockStats.collisionAvoidanceChecks);
                    ImGui::Text("Cells/nodes      : %d", flockStats.spatialCellsOrNodes);
                    ImGui::Text("Memory estimate  : %.1f KB",
                        static_cast<float>(flockStats.memoryEstimateBytes) / 1024.0f);
                    ImGui::Text("Update time      : %.3f ms", flockStats.updateMs);
                    ImGui::Text("Spatial build    : %.3f ms", flockStats.spatialBuildMs);
                    ImGui::Text("Neighbour search : %.3f ms", flockStats.neighbourSearchMs);
                }
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

                    float correctionStrength = shared.replicaCorrectionStrength.load(std::memory_order_relaxed);
                    if (ImGui::SliderFloat("Correction Strength", &correctionStrength, 0.0f, 3.0f, "%.2f"))
                        shared.replicaCorrectionStrength.store(correctionStrength, std::memory_order_relaxed);
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
                    ImGui::Text("Stale replica updates     : %u",
                        shared.staleReplicaPacketsIgnored.load(std::memory_order_relaxed));

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

                ImGui::SeparatorText("Peer / Object Debug");
                ImGui::Text("Local peer/client ID       : %d", cfg.peer_id);
                ImGui::Text("Configured remote peers    : %d", (int)cfg.RemotePeers().size());
                for (const auto& peer : cfg.RemotePeers())
                {
                    ImGui::Text("Peer %d -> %s  control:%u  snapshot:%u",
                        peer.peerId,
                        peer.host.c_str(),
                        (unsigned)peer.control_port,
                        (unsigned)peer.snapshot_port);
                }
                {
                    RuntimeDebugCounts counts{};
                    {
                        std::lock_guard<std::mutex> lk(shared.debugCountsMutex);
                        counts = shared.debugCounts;
                    }

                    ImGui::Text("Local-owned objects        : %u", counts.localOwned);
                    ImGui::Text("Remote-owned objects       : %u", counts.remoteOwned);
                    ImGui::Text("Static/local objects       : %u", counts.staticObjects);
                    ImGui::Text("Animated objects           : %u", counts.animatedObjects);
                    ImGui::Text("Spawned objects approx     : %u", counts.spawnedObjects);
                    ImGui::Text("Dynamic body cache         : %u", counts.dynamicBodies);
                    ImGui::Text("Collision candidate pairs  : %u", counts.collisionCandidates);
                    ImGui::Text("  solid/solid              : %u", counts.collisionSolidSolidCandidates);
                    ImGui::Text("  solid/container          : %u", counts.collisionSolidContainerCandidates);
                    ImGui::Text("Collision contacts         : %u", counts.collisionContacts);
                }

                ImGui::Separator();

                ImGui::SeparatorText("Thread / Core Mapping");
                ImGui::Text("Available cores : %d", numCores);
                ImGui::Text("Render  thread  -> logical core %d", shared.renderCoreAssigned);
                ImGui::Text("Network affinity -> logical processors %d-%d",
                    shared.netSendCoreAssigned,
                    shared.netRecvCoreAssigned);
                ImGui::Text("Network send    -> logical core %d (%s)",
                    shared.netSendCoreAssigned,
                    shared.netSendThreadRunning.load(std::memory_order_relaxed) ? "running" : "stopped");
                ImGui::Text("Network receive -> logical core %d (%s)",
                    shared.netRecvCoreAssigned,
                    shared.netRecvThreadRunning.load(std::memory_order_relaxed) ? "running" : "stopped");
                ImGui::Text("Sim     thread  -> logical core %d", shared.simCoreAssigned);
                ImGui::Text("Sim workers     : %d", shared.simWorkerThreadCount.load(std::memory_order_relaxed));
                {
                    std::vector<int> simCores;
                    {
                        std::lock_guard<std::mutex> lk(shared.simWorkerCoresMutex);
                        simCores = shared.simWorkerCores;
                    }

                    std::ostringstream oss;
                    for (size_t i = 0; i < simCores.size(); ++i)
                    {
                        if (i > 0) oss << ", ";
                        oss << simCores[i];
                    }
                    ImGui::Text("Sim core range  -> logical core(s) %s", oss.str().c_str());
                }
                ImGui::Separator();

                if (snap)
                    ImGui::Text("Snapshot tick   : %llu", (unsigned long long)snap->simTickNumber);
                else
                    ImGui::TextDisabled("No snapshot yet");

                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
        }

        {
            sceneCameraOptions = CollectSceneCameras(world);

            Net::NetworkStats netStats{};
            {
                std::lock_guard<std::mutex> lk(shared.netStatsMutex);
                netStats = shared.netStats;
            }

            SimTimingStats timingStats{};
            {
                std::lock_guard<std::mutex> lk(shared.simTimingMutex);
                timingStats = shared.simTiming;
            }

            RuntimeDebugCounts counts{};
            {
                std::lock_guard<std::mutex> lk(shared.debugCountsMutex);
                counts = shared.debugCounts;
            }

            FlockingStats flockStats{};
            {
                std::lock_guard<std::mutex> lk(shared.flockingStatsMutex);
                flockStats = shared.flockingStats;
            }

            std::vector<Net::PeerDebugInfo> peerDebugInfo;
            {
                std::lock_guard<std::mutex> lk(shared.peerDebugMutex);
                peerDebugInfo = shared.peerDebugInfo;
            }

            std::vector<int> simCores;
            {
                std::lock_guard<std::mutex> lk(shared.simWorkerCoresMutex);
                simCores = shared.simWorkerCores;
            }
            std::ostringstream simCoreText;
            if (simCores.empty())
            {
                simCoreText << shared.simCoreAssigned;
            }
            else
            {
                for (size_t i = 0; i < simCores.size(); ++i)
                {
                    if (i > 0) simCoreText << ", ";
                    simCoreText << simCores[i];
                }
            }

            static std::array<FlockingStats, 4> flockComparisonSamples{};
            static std::array<bool, 4> flockComparisonValid{ false, false, false, false };
            const int sampleMode = static_cast<int>(flockStats.searchMode);
            if (sampleMode >= 0 && sampleMode < static_cast<int>(flockComparisonSamples.size()) && flockStats.agentCount > 0)
            {
                flockComparisonSamples[static_cast<size_t>(sampleMode)] = flockStats;
                flockComparisonValid[static_cast<size_t>(sampleMode)] = true;
            }

            const int currentSceneIndex = scenarios.CurrentIndex();
            const char* currentSceneName = scenarios.Current()
                ? scenarios.Current()->Name()
                : "None";

            std::string currentCameraName = "Free Camera";
            CameraComponent::Projection currentCameraProjection = renderCamera.projection;
            for (const auto& camOption : sceneCameraOptions)
            {
                if (camOption.entity == activeSceneCamera)
                {
                    currentCameraName = camOption.name;
                    currentCameraProjection = camOption.projection;
                    break;
                }
            }

            auto requestSceneSwitch = [&](int sceneIndex, bool forceReload = false)
                {
                    if ((!forceReload && sceneIndex == currentSceneIndex) || sceneIndex < 0 || sceneIndex >= scenarios.Count())
                        return;

                    const uint32_t newGeneration =
                        shared.sceneGeneration.fetch_add(1, std::memory_order_acq_rel) + 1;

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
                        shared.inReplicaLatestTick.clear();
                    }
                    {
                        std::lock_guard<std::mutex> lk(shared.inSpawnMutex);
                        shared.inSpawnEvents.clear();
                    }

                    shared.sceneTransitionActive.store(true, std::memory_order_release);
                    shared.sceneTransitionRemainingSec.store(0.25f, std::memory_order_release);
                    shared.requestedSceneReload.store(forceReload, std::memory_order_release);
                    shared.requestedSceneIndex.store(sceneIndex, std::memory_order_release);
                    activeSceneCamera = INVALID_ENTITY;

                    shared.sendSceneGeneration.store(newGeneration, std::memory_order_release);
                    shared.sendSceneIndex.store(sceneIndex, std::memory_order_relaxed);
                    shared.sendSceneChange.store(true, std::memory_order_release);
                };

            auto showTimingRow = [](const char* name, float avgMs, float maxMs)
                {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(name);
                    ImGui::TableSetColumnIndex(1); ImGui::Text("%.3f", avgMs);
                    ImGui::TableSetColumnIndex(2); ImGui::Text("%.3f", maxMs);
                };

            if (ImGui::BeginMainMenuBar())
            {
            ImGui::Text("Final Lab");
            ImGui::Separator();

            if (ImGui::BeginMenu("Summary"))
            {
                ImGui::Text("Scene: %s", currentSceneName);
                ImGui::Text("Peer: %d / 4", cfg.peer_id);
                ImGui::Text("Display Mode: %s", DisplayModeName(shared.displayMode.load(std::memory_order_relaxed)));
                ImGui::Text("Camera: %s (%s)", currentCameraName.c_str(), CameraProjectionName(currentCameraProjection));

            if (ImGui::BeginTable("SummaryHzTable", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
            {
                ImGui::TableSetupColumn("System");
                ImGui::TableSetupColumn("Target Hz");
                ImGui::TableSetupColumn("Measured Hz");
                ImGui::TableHeadersRow();

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Render");
                ImGui::TableSetColumnIndex(1); ImGui::Text("%.0f", shared.targetRenderHz.load(std::memory_order_relaxed));
                ImGui::TableSetColumnIndex(2); ImGui::Text("%.1f", displayRenderHz);

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Network Send");
                ImGui::TableSetColumnIndex(1); ImGui::Text("%.0f", shared.targetNetworkHz.load(std::memory_order_relaxed));
                ImGui::TableSetColumnIndex(2); ImGui::Text("%.1f", shared.measuredNetworkSendHz.load(std::memory_order_relaxed));

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Network Receive");
                ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted("polling");
                ImGui::TableSetColumnIndex(2); ImGui::Text("%.1f", shared.measuredNetworkRecvHz.load(std::memory_order_relaxed));

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Fixed Simulation Step");
                ImGui::TableSetColumnIndex(1); ImGui::Text("%.0f", shared.targetSimHz.load(std::memory_order_relaxed));
                ImGui::TableSetColumnIndex(2); ImGui::Text("%.1f", displaySimHz);

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted("Simulation Outer Loop");
                ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted("as needed");
                ImGui::TableSetColumnIndex(2); ImGui::Text("%.1f", shared.measuredSimLoopHz.load(std::memory_order_relaxed));

                ImGui::EndTable();
            }

                ImGui::Text("Render core: logical %d", shared.renderCoreAssigned);
                ImGui::Text("Network recv core: logical %d", shared.netRecvCoreAssigned);
                ImGui::Text("Network send core: logical %d", shared.netSendCoreAssigned);
                ImGui::Text("Simulation cores: logical %s", simCoreText.str().c_str());
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Scene / Simulation"))
            {
                ImGui::Text("Current scene: %s", currentSceneName);
                ImGui::TextDisabled("Scene switching is Global. Camera and display mode are Local.");

                bool simRunning = shared.simRunning.load(std::memory_order_relaxed);
                if (ImGui::MenuItem(simRunning ? "Pause Simulation" : "Start Simulation"))
                    shared.simRunning.store(!simRunning, std::memory_order_relaxed);
                if (ImGui::MenuItem("Step Once"))
                    shared.stepOnce.store(true, std::memory_order_relaxed);

                bool useFixed = shared.useFixedTimestep.load(std::memory_order_relaxed);
                if (ImGui::Checkbox("Use Fixed Timestep", &useFixed))
                    shared.useFixedTimestep.store(useFixed, std::memory_order_relaxed);

                float fdt = shared.fixedDt.load(std::memory_order_relaxed);
                float oldFdt = fdt;
                ImGui::InputFloat("Fixed dt (s)", &fdt, 0.001f, 0.01f, "%.4f");
                fdt = std::max(0.0001f, std::min(fdt, 0.1f));
                if (fdt != oldFdt)
                    shared.fixedDt.store(fdt, std::memory_order_relaxed);
                ImGui::Text("Fixed timestep frequency: %.1f Hz", 1.0f / fdt);

                const char* integrators[] = { "Euler", "Semi-Implicit Euler" };
                int integ = shared.integratorType.load(std::memory_order_relaxed);
                if (ImGui::Combo("Integrator", &integ, integrators, IM_ARRAYSIZE(integrators)))
                    shared.integratorType.store(integ, std::memory_order_relaxed);

                ImGui::SeparatorText("Global Scene Controls");
                if (ImGui::BeginCombo("Scene (Global)", currentSceneName))
                {
                    for (int si = 0; si < scenarios.Count(); ++si)
                    {
                        const bool selected = (si == currentSceneIndex);
                        if (ImGui::Selectable(scenarios.Get(si)->Name(), selected))
                            requestSceneSwitch(si);
                        if (selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
                bool grav = shared.gravityOn.load(std::memory_order_relaxed);
                if (ImGui::Checkbox("Gravity Enabled (Global)", &grav))
                {
                    shared.pendingGravityEnabled.store(grav, std::memory_order_relaxed);
                    shared.pendingGravityChange.store(true, std::memory_order_release);
                    shared.sendGravityEnabled.store(grav, std::memory_order_relaxed);
                    shared.sendGravityChange.store(true, std::memory_order_release);
                }

                const char* displayModes[] = { "Material Colours", "Owner Colours" };
                int displayMode = shared.displayMode.load(std::memory_order_relaxed);
                if (ImGui::Combo("Display Mode (Local)", &displayMode, displayModes, IM_ARRAYSIZE(displayModes)))
                    shared.displayMode.store(displayMode, std::memory_order_relaxed);

                {
                    std::lock_guard<std::mutex> lk(shared.clearColourMutex);
                    ImGui::ColorEdit3("Background Colour (Local)", shared.clearColour);
                }

                if (ImGui::BeginCombo("Camera (Local)", currentCameraName.c_str()))
                {
                    for (const auto& camOption : sceneCameraOptions)
                    {
                        std::string label = camOption.name + " [" + CameraProjectionName(camOption.projection) + "]";
                        const bool selected = (activeSceneCamera == camOption.entity);
                        if (ImGui::Selectable(label.c_str(), selected))
                        {
                            if (ApplySceneCamera(world, camOption.entity, renderCamera))
                                activeSceneCamera = camOption.entity;
                        }
                        if (selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
                ImGui::Text("Camera info: %s  FOV %.1f  Ortho %.1f  Near/Far %.2f / %.1f",
                    CameraProjectionName(renderCamera.projection),
                    renderCamera.fov,
                    renderCamera.orthoSize,
                    renderCamera.nearClip,
                    renderCamera.farClip);
                ImGui::TextDisabled("Controls: WASD/QE move, arrow keys look. Up looks up, Down looks down.");
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Networking / Peers"))
            {
                ImGui::Text("Architecture: peer-to-peer, no client-server authority");
                ImGui::Text("Local peer/client ID: %d", cfg.peer_id);
                ImGui::Text("Configured remote peers: %d", (int)cfg.RemotePeers().size());
                ImGui::Text("Control port: %u  Snapshot port: %u",
                    (unsigned)cfg.control_bind_port,
                    (unsigned)cfg.snapshot_bind_port);
                for (const auto& peer : cfg.RemotePeers())
                {
                    ImGui::Text("Peer %d -> %s  control:%u  snapshot:%u",
                        peer.peerId,
                        peer.host.c_str(),
                        (unsigned)peer.control_port,
                        (unsigned)peer.snapshot_port);
                }

                if (ImGui::BeginTable("NetworkStatsTable", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
                {
                    ImGui::TableSetupColumn("Metric");
                    ImGui::TableSetupColumn("Value");
                    ImGui::TableHeadersRow();
#define NET_ROW(label, value) ImGui::TableNextRow(); ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(label); ImGui::TableSetColumnIndex(1); ImGui::Text("%u", (unsigned)(value))
                    NET_ROW("Control packets received", netStats.controlPacketsReceived);
                    NET_ROW("Snapshot packets sent", netStats.snapshotPacketsSent);
                    NET_ROW("Snapshots sent on control fallback", netStats.snapshotPacketsSentOnControl);
                    NET_ROW("Snapshot send failures", netStats.snapshotPacketsSendFailed);
                    NET_ROW("Snapshot packets received", netStats.snapshotPacketsReceived);
                    NET_ROW("Snapshot packets dropped", netStats.snapshotPacketsDropped);
                    NET_ROW("Snapshot packets delayed", netStats.snapshotPacketsDelayed);
                    NET_ROW("Snapshots skipped inactive peer", netStats.snapshotPacketsSkippedInactivePeer);
                    NET_ROW("Global commands sent", netStats.globalCommandsSent);
                    NET_ROW("Global commands received", netStats.globalCommandsReceived);
                    NET_ROW("Spawn packets sent", netStats.spawnPacketsSent);
                    NET_ROW("Spawn packets received", netStats.spawnPacketsReceived);
                    NET_ROW("Reliable resends", netStats.reliableResends);
                    NET_ROW("Discovery packets sent", netStats.discoveryPacketsSent);
                    NET_ROW("Discovery packets received", netStats.discoveryPacketsReceived);
                    NET_ROW("Peer discovery updates", netStats.peersDiscovered);
                    NET_ROW("Endpoints learned from control", netStats.endpointsLearnedFromControl);
                    NET_ROW("Stale replica updates ignored", shared.staleReplicaPacketsIgnored.load(std::memory_order_relaxed));
#undef NET_ROW
                    ImGui::EndTable();
                }

                ImGui::SeparatorText("Peer Latency / Wi-Fi Debug");
                ImGui::TextWrapped("RTT is measured with lightweight peer-to-peer ping/pong packets on the control socket.");
                if (ImGui::BeginTable("PeerLatencyTable", 10, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
                {
                    ImGui::TableSetupColumn("Peer");
                    ImGui::TableSetupColumn("Active");
                    ImGui::TableSetupColumn("HELLO");
                    ImGui::TableSetupColumn("WELCOME");
                    ImGui::TableSetupColumn("Last RTT");
                    ImGui::TableSetupColumn("Avg RTT");
                    ImGui::TableSetupColumn("Jitter");
                    ImGui::TableSetupColumn("Sent");
                    ImGui::TableSetupColumn("Replies");
                    ImGui::TableSetupColumn("Timeouts");
                    ImGui::TableHeadersRow();

                    for (const auto& peerInfo : peerDebugInfo)
                    {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0); ImGui::Text("%d", peerInfo.peerId);
                        ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(peerInfo.active ? "yes" : "no");
                        ImGui::TableSetColumnIndex(2); ImGui::TextUnformatted(peerInfo.helloReceived ? "yes" : "no");
                        ImGui::TableSetColumnIndex(3); ImGui::TextUnformatted(peerInfo.welcomeReceived ? "yes" : "no");
                        ImGui::TableSetColumnIndex(4);
                        if (peerInfo.lastRttMs >= 0.0) ImGui::Text("%.1f ms", peerInfo.lastRttMs);
                        else ImGui::TextUnformatted("--");
                        ImGui::TableSetColumnIndex(5);
                        if (peerInfo.avgRttMs >= 0.0) ImGui::Text("%.1f ms", peerInfo.avgRttMs);
                        else ImGui::TextUnformatted("--");
                        ImGui::TableSetColumnIndex(6); ImGui::Text("%.1f ms", peerInfo.jitterMs);
                        ImGui::TableSetColumnIndex(7); ImGui::Text("%u", peerInfo.pingsSent);
                        ImGui::TableSetColumnIndex(8); ImGui::Text("%u", peerInfo.pongsReceived);
                        ImGui::TableSetColumnIndex(9); ImGui::Text("%u", peerInfo.pingsTimedOut);
                    }

                    if (peerDebugInfo.empty())
                    {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextDisabled("No configured remote peers");
                    }

                    ImGui::EndTable();
                }
                ImGui::TextDisabled("Guide: wired/LAN should be low and steady; Wi-Fi trouble usually shows higher jitter or timeouts.");

                ImGui::SeparatorText("Network Impairment (Local)");
                bool impairOn = shared.netImpairmentEnabled.load(std::memory_order_relaxed);
                if (ImGui::Checkbox("Enable Snapshot Impairment", &impairOn))
                    shared.netImpairmentEnabled.store(impairOn, std::memory_order_relaxed);
                float latencyMs = shared.netLatencyMs.load(std::memory_order_relaxed);
                if (ImGui::SliderFloat("Latency (ms)", &latencyMs, 0.0f, 300.0f, "%.0f"))
                    shared.netLatencyMs.store(latencyMs, std::memory_order_relaxed);
                float jitterMs = shared.netJitterMs.load(std::memory_order_relaxed);
                if (ImGui::SliderFloat("Jitter (ms)", &jitterMs, 0.0f, 150.0f, "%.0f"))
                    shared.netJitterMs.store(jitterMs, std::memory_order_relaxed);
                float dropPct = shared.netDropPercent.load(std::memory_order_relaxed);
                if (ImGui::SliderFloat("Packet Loss (%)", &dropPct, 0.0f, 80.0f, "%.0f"))
                    shared.netDropPercent.store(dropPct, std::memory_order_relaxed);
                if (ImGui::Button("Coursework Worst Case: 100ms +/-50ms, 20% loss"))
                {
                    shared.netImpairmentEnabled.store(true, std::memory_order_relaxed);
                    shared.netLatencyMs.store(100.0f, std::memory_order_relaxed);
                    shared.netJitterMs.store(50.0f, std::memory_order_relaxed);
                    shared.netDropPercent.store(20.0f, std::memory_order_relaxed);
                }

                ImGui::SeparatorText("Replica Smoothing / Interpolation");
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

                float correctionStrength = shared.replicaCorrectionStrength.load(std::memory_order_relaxed);
                if (ImGui::SliderFloat("Correction Strength", &correctionStrength, 0.0f, 3.0f, "%.2f"))
                    shared.replicaCorrectionStrength.store(correctionStrength, std::memory_order_relaxed);
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Concurrency / Threads"))
            {
                if (ImGui::BeginTable("ConcurrencyTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
                {
                    ImGui::TableSetupColumn("System");
                    ImGui::TableSetupColumn("Target Hz");
                    ImGui::TableSetupColumn("Measured Hz");
                    ImGui::TableSetupColumn("Thread/Core");
                    ImGui::TableHeadersRow();

                    auto freqRow = [](const char* system, const char* target, float measured, const char* core)
                        {
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(system);
                            ImGui::TableSetColumnIndex(1); ImGui::TextUnformatted(target);
                            ImGui::TableSetColumnIndex(2); ImGui::Text("%.1f", measured);
                            ImGui::TableSetColumnIndex(3); ImGui::TextUnformatted(core);
                        };

                    char renderTarget[32], netTarget[32], simTarget[32], renderCoreText[32], recvCoreText[32], sendCoreText[32], simCoreBuf[128];
                    snprintf(renderTarget, sizeof(renderTarget), "%.0f", shared.targetRenderHz.load(std::memory_order_relaxed));
                    snprintf(netTarget, sizeof(netTarget), "%.0f", shared.targetNetworkHz.load(std::memory_order_relaxed));
                    snprintf(simTarget, sizeof(simTarget), "%.0f", shared.targetSimHz.load(std::memory_order_relaxed));
                    snprintf(renderCoreText, sizeof(renderCoreText), "logical %d", shared.renderCoreAssigned);
                    snprintf(recvCoreText, sizeof(recvCoreText), "logical %d", shared.netRecvCoreAssigned);
                    snprintf(sendCoreText, sizeof(sendCoreText), "logical %d", shared.netSendCoreAssigned);
                    snprintf(simCoreBuf, sizeof(simCoreBuf), "logical %s", simCoreText.str().c_str());

                    freqRow("Render", renderTarget, displayRenderHz, renderCoreText);
                    freqRow("Network Receive", "polling", shared.measuredNetworkRecvHz.load(std::memory_order_relaxed), recvCoreText);
                    freqRow("Network Send", netTarget, shared.measuredNetworkSendHz.load(std::memory_order_relaxed), sendCoreText);
                    freqRow("Fixed Simulation Step", simTarget, displaySimHz, simCoreBuf);
                    freqRow("Simulation Outer Loop", "as needed", shared.measuredSimLoopHz.load(std::memory_order_relaxed), simCoreBuf);

                    ImGui::EndTable();
                }

                if (ImGui::SliderFloat("Render Hz##Panel", &uiRenderHz, 0.0f, 300.0f, "%.0f"))
                    shared.targetRenderHz.store(uiRenderHz, std::memory_order_relaxed);
                if (ImGui::SliderFloat("Network Send Hz##Panel", &uiNetHz, 0.0f, 120.0f, "%.0f"))
                    shared.targetNetworkHz.store(uiNetHz, std::memory_order_relaxed);
                if (ImGui::SliderFloat("Simulation Hz##Panel", &uiSimHz, 0.0f, 2000.0f, "%.0f"))
                {
                    shared.targetSimHz.store(uiSimHz, std::memory_order_relaxed);
                    if (uiSimHz > 0.0f)
                        shared.fixedDt.store(1.0f / uiSimHz, std::memory_order_relaxed);
                }
                if (ImGui::SliderFloat("Snapshot Send Hz##Panel", &uiSnapshotHz, 0.0f, 2000.0f, "%.0f"))
                {
                    const float minSnapshotHz = std::max(1.0f, shared.targetNetworkHz.load(std::memory_order_relaxed));
                    if (uiSnapshotHz > 0.0f && uiSnapshotHz < minSnapshotHz)
                        uiSnapshotHz = minSnapshotHz;
                    shared.snapshotSendHz.store(uiSnapshotHz, std::memory_order_relaxed);
                }
                ImGui::Text("Snapshot rule: %s",
                    uiSnapshotHz <= 0.0f
                    ? "every fresh simulation tick"
                    : "rate limited, minimum network tick rate");
                ImGui::Text("Fixed timestep: %.6f s (%.1f Hz)",
                    shared.fixedDt.load(std::memory_order_relaxed),
                    1.0f / std::max(0.000001f, shared.fixedDt.load(std::memory_order_relaxed)));
                ImGui::Text("Fixed steps last loop: %u / %u",
                    shared.fixedStepsLast.load(std::memory_order_relaxed),
                    shared.fixedStepsMaxAllowed.load(std::memory_order_relaxed));
                ImGui::Text("Accumulator before/after/dropped: %.3f / %.3f / %.3f ms",
                    shared.accumulatorBeforeMs.load(std::memory_order_relaxed),
                    shared.accumulatorAfterMs.load(std::memory_order_relaxed),
                    shared.accumulatorDroppedMs.load(std::memory_order_relaxed));
                ImGui::Text("Simulation workers: %d", shared.simWorkerThreadCount.load(std::memory_order_relaxed));

                ImGui::SeparatorText("Required Processor Mapping");
                ImGui::Text("Spec core 1 -> logical processor 0 -> Render");
                ImGui::Text("Spec core 2 -> logical processor 1 -> Network Receive");
                ImGui::Text("Spec core 3 -> logical processor 2 -> Network Send");
                ImGui::Text("Spec core 4+ -> logical processor 3+ -> Simulation");
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Scene Objects / Physics"))
            {
                const uint32_t totalOwnedObjects =
                    counts.localOwned + counts.remoteOwned + counts.staticObjects + counts.animatedObjects;
                ImGui::Text("Rendered instances: %u", snap ? (unsigned)snap->instances.size() : 0u);
                ImGui::Text("Tracked scene objects: %u", totalOwnedObjects);
                ImGui::Text("Static/local objects: %u", counts.staticObjects);
                ImGui::Text("Animated objects: %u", counts.animatedObjects);
                ImGui::Text("Local-owned simulated objects: %u", counts.localOwned);
                ImGui::Text("Remote replicated objects: %u", counts.remoteOwned);
                ImGui::Text("Spawned objects approx: %u", counts.spawnedObjects);
                ImGui::Text("Flocking boids: %d", flockStats.agentCount);

                ImGui::SeparatorText("Physics / Collision");
                ImGui::Text("Dynamic body cache: %u", counts.dynamicBodies);
                ImGui::Text("Collision narrow-phase pairs: %u", counts.collisionCandidates);
                ImGui::Text("  Solid/solid pairs: %u", counts.collisionSolidSolidCandidates);
                ImGui::Text("  Solid/container pairs: %u", counts.collisionSolidContainerCandidates);
                ImGui::Text("Broad-phase rejected pairs: %u", counts.collisionBroadPhaseRejected);
                ImGui::Text("  Container rejects: %u", counts.collisionContainerBroadPhaseRejected);
                ImGui::Text("Spatial broad-phase cells: %u", counts.collisionSpatialCells);
                ImGui::Text("Collision contacts: %u", counts.collisionContacts);
                ImGui::Text("Solver velocity iterations: %u", counts.collisionSolverVelocityIterations);
                ImGui::Text("Physics update avg/max: %.3f / %.3f ms", timingStats.physicsMsAvg, timingStats.physicsMsMax);
                ImGui::Text("Collision update avg/max: %.3f / %.3f ms", timingStats.collisionMsAvg, timingStats.collisionMsMax);
                ImGui::Text("Scenario/spawner avg/max: %.3f / %.3f ms", timingStats.scenarioMsAvg, timingStats.scenarioMsMax);
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Flocking / Advanced Feature"))
            {
                if (flockStats.agentCount <= 0)
                    ImGui::TextDisabled("No flocking agents in current scene.");

                bool flockingEnabled = shared.flockingEnabled.load(std::memory_order_relaxed);
                if (ImGui::Checkbox("Flocking Enabled", &flockingEnabled))
                    shared.flockingEnabled.store(flockingEnabled, std::memory_order_relaxed);
                bool boundsEnabled = shared.flockingBoundsEnabled.load(std::memory_order_relaxed);
                if (ImGui::Checkbox("Bounds Avoidance Enabled", &boundsEnabled))
                    shared.flockingBoundsEnabled.store(boundsEnabled, std::memory_order_relaxed);
                bool debugEnabled = shared.flockingDebugEnabled.load(std::memory_order_relaxed);
                if (ImGui::Checkbox("Debug Force Data / Visualisation Enabled", &debugEnabled))
                    shared.flockingDebugEnabled.store(debugEnabled, std::memory_order_relaxed);
                bool useUiSettings = shared.flockingUseUiSettings.load(std::memory_order_relaxed);
                if (ImGui::Checkbox("Apply UI Settings To Boids", &useUiSettings))
                    shared.flockingUseUiSettings.store(useUiSettings, std::memory_order_relaxed);

                if (auto* fbScene = dynamic_cast<Scenario_FlatbufferScene*>(scenarios.Current()))
                {
                    if (fbScene->HasFlockingAgents() || flockStats.agentCount > 0)
                    {
                        static int flatbufferBoidTarget = 100;
                        static int lastObservedFlockingCount = 0;
                        if (flockStats.agentCount > 0 && flockStats.agentCount != lastObservedFlockingCount)
                        {
                            flatbufferBoidTarget = flockStats.agentCount;
                            lastObservedFlockingCount = flockStats.agentCount;
                        }

                        ImGui::SeparatorText("FlatBuffer Flocking Scene");
                        ImGui::SliderInt("Target Boid Count", &flatbufferBoidTarget, 10, 2000);
                        ImGui::SameLine();
                        if (ImGui::Button("Apply Count"))
                            fbScene->RequestFlockingBoidCount(flatbufferBoidTarget);
                    }
                }

                int searchMode = shared.flockingSearchMode.load(std::memory_order_relaxed);
                const char* searchModeLabels[] = {
                    "Brute Force (baseline, no segmentation)",
                    "Uniform Grid (spatial segmentation 1)",
                    "Octree (spatial segmentation 2)",
                    "GPU Compute Brute Force"
                };
                if (ImGui::Combo("Neighbour Search Mode", &searchMode, searchModeLabels, IM_ARRAYSIZE(searchModeLabels)))
                    shared.flockingSearchMode.store(searchMode, std::memory_order_relaxed);

                float maxSpeed = shared.flockingMaxSpeed.load(std::memory_order_relaxed);
                if (ImGui::SliderFloat("Max Speed##FlockPanel", &maxSpeed, 0.5f, 30.0f, "%.1f"))
                    shared.flockingMaxSpeed.store(maxSpeed, std::memory_order_relaxed);
                float maxForce = shared.flockingMaxForce.load(std::memory_order_relaxed);
                if (ImGui::SliderFloat("Max Force##FlockPanel", &maxForce, 0.5f, 60.0f, "%.1f"))
                    shared.flockingMaxForce.store(maxForce, std::memory_order_relaxed);
                float perceptionRadius = shared.flockingPerceptionRadius.load(std::memory_order_relaxed);
                if (ImGui::SliderFloat("Perception Radius##FlockPanel", &perceptionRadius, 0.5f, 20.0f, "%.1f"))
                    shared.flockingPerceptionRadius.store(perceptionRadius, std::memory_order_relaxed);
                float separationRadius = shared.flockingSeparationRadius.load(std::memory_order_relaxed);
                if (ImGui::SliderFloat("Separation Radius##FlockPanel", &separationRadius, 0.1f, 10.0f, "%.1f"))
                    shared.flockingSeparationRadius.store(std::min(separationRadius, shared.flockingPerceptionRadius.load(std::memory_order_relaxed)), std::memory_order_relaxed);

                ImGui::SeparatorText("Weighted Truncated Sum");
                float avoidanceWeight = shared.flockingAvoidanceWeight.load(std::memory_order_relaxed);
                if (ImGui::SliderFloat("Avoidance Weight##FlockPanel", &avoidanceWeight, 0.0f, 8.0f, "%.2f"))
                    shared.flockingAvoidanceWeight.store(avoidanceWeight, std::memory_order_relaxed);
                float separationWeight = shared.flockingSeparationWeight.load(std::memory_order_relaxed);
                if (ImGui::SliderFloat("Separation Weight##FlockPanel", &separationWeight, 0.0f, 8.0f, "%.2f"))
                    shared.flockingSeparationWeight.store(separationWeight, std::memory_order_relaxed);
                float alignmentWeight = shared.flockingAlignmentWeight.load(std::memory_order_relaxed);
                if (ImGui::SliderFloat("Alignment Weight##FlockPanel", &alignmentWeight, 0.0f, 8.0f, "%.2f"))
                    shared.flockingAlignmentWeight.store(alignmentWeight, std::memory_order_relaxed);
                float cohesionWeight = shared.flockingCohesionWeight.load(std::memory_order_relaxed);
                if (ImGui::SliderFloat("Cohesion Weight##FlockPanel", &cohesionWeight, 0.0f, 8.0f, "%.2f"))
                    shared.flockingCohesionWeight.store(cohesionWeight, std::memory_order_relaxed);

                ImGui::SeparatorText("Live Flocking Stats");
                ImGui::Text("Agents: %d  Locally updated: %d  Debug agents: %d",
                    flockStats.agentCount,
                    flockStats.ownedAgentCount,
                    shared.flockingDebugAgentCount.load(std::memory_order_relaxed));
                ImGui::Text("Mode: %s", FlockingSearchModeName(flockStats.searchMode));
                if (flockStats.searchMode == FlockingNeighbourSearchMode::GpuComputeBruteForce)
                {
                    ImGui::Text("GPU Compute: %s", flockStats.gpuComputeAvailable ? "available" : "unavailable");
                    if (flockStats.gpuFallbackActive)
                        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.25f, 1.0f), "CPU fallback active");
                    ImGui::Text("GPU upload/dispatch/readback: %.3f / %.3f / %.3f ms",
                        flockStats.gpuUploadMs,
                        flockStats.gpuDispatchMs,
                        flockStats.gpuReadbackMs);
                    ImGui::Text("GPU total: %.3f ms", flockStats.gpuTotalMs);
                }
                ImGui::Text("Neighbour checks: %d  Spatial candidates: %d  Found: %d",
                    flockStats.neighbourChecks,
                    flockStats.spatialCandidateChecks,
                    flockStats.neighboursFound);
                ImGui::Text("Collision avoidance checks: %d", flockStats.collisionAvoidanceChecks);
                ImGui::Text("Cells/nodes: %d  Memory estimate: %.1f KB",
                    flockStats.spatialCellsOrNodes,
                    static_cast<float>(flockStats.memoryEstimateBytes) / 1024.0f);
                ImGui::Text("Spatial build: %.3f ms  Search: %.3f ms  Total: %.3f ms",
                    flockStats.spatialBuildMs,
                    flockStats.neighbourSearchMs,
                    flockStats.updateMs);

                ImGui::SeparatorText("Spatial Segmentation Comparison");
                if (ImGui::Button("Record Current Mode Sample"))
                {
                    if (sampleMode >= 0 && sampleMode < static_cast<int>(flockComparisonSamples.size()))
                    {
                        flockComparisonSamples[static_cast<size_t>(sampleMode)] = flockStats;
                        flockComparisonValid[static_cast<size_t>(sampleMode)] = true;
                    }
                }
                if (ImGui::BeginTable("FlockingComparisonTable", 9, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
                {
                    ImGui::TableSetupColumn("Mode");
                    ImGui::TableSetupColumn("Boids");
                    ImGui::TableSetupColumn("Neighbour Checks");
                    ImGui::TableSetupColumn("Spatial Candidates");
                    ImGui::TableSetupColumn("Build ms");
                    ImGui::TableSetupColumn("Search ms");
                    ImGui::TableSetupColumn("Update ms");
                    ImGui::TableSetupColumn("Memory KB");
                    ImGui::TableSetupColumn("Cells/Nodes");
                    ImGui::TableHeadersRow();

                    for (int i = 0; i < static_cast<int>(flockComparisonSamples.size()); ++i)
                    {
                        const FlockingStats& s = flockComparisonSamples[static_cast<size_t>(i)];
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(FlockingSearchModeName(static_cast<FlockingNeighbourSearchMode>(i)));
                        if (!flockComparisonValid[static_cast<size_t>(i)])
                        {
                            ImGui::TableSetColumnIndex(1); ImGui::TextDisabled("no sample");
                            continue;
                        }
                        ImGui::TableSetColumnIndex(1); ImGui::Text("%d", s.agentCount);
                        ImGui::TableSetColumnIndex(2); ImGui::Text("%d", s.neighbourChecks);
                        ImGui::TableSetColumnIndex(3); ImGui::Text("%d", s.spatialCandidateChecks);
                        ImGui::TableSetColumnIndex(4); ImGui::Text("%.3f", s.spatialBuildMs);
                        ImGui::TableSetColumnIndex(5); ImGui::Text("%.3f", s.neighbourSearchMs);
                        ImGui::TableSetColumnIndex(6); ImGui::Text("%.3f", s.updateMs);
                        ImGui::TableSetColumnIndex(7); ImGui::Text("%.1f", static_cast<float>(s.memoryEstimateBytes) / 1024.0f);
                        ImGui::TableSetColumnIndex(8); ImGui::Text("%d", s.spatialCellsOrNodes);
                    }
                    ImGui::EndTable();
                }
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Debug / Performance"))
            {
                ImGui::Text("Frame time: %.3f ms", displayRenderHz > 0.0f ? 1000.0f / displayRenderHz : 0.0f);
                ImGui::Text("Fixed steps last loop: %u / %u",
                    timingStats.fixedStepsLast,
                    timingStats.fixedStepsMaxAllowed);
                ImGui::Text("Accumulator before stepping: %.3f ms", timingStats.accumulatorBeforeMs);
                ImGui::Text("Accumulator after stepping : %.3f ms", timingStats.accumulatorAfterMs);
                ImGui::Text("Dropped accumulator time   : %.3f ms", timingStats.accumulatorDroppedMs);
                ImGui::Text("Estimated sleep/wait time  : %.3f ms", timingStats.estimatedSleepWaitMs);
                if (ImGui::BeginTable("TimingStatsTable", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
                {
                    ImGui::TableSetupColumn("Section");
                    ImGui::TableSetupColumn("Avg ms");
                    ImGui::TableSetupColumn("Max ms");
                    ImGui::TableHeadersRow();
                    showTimingRow("Scenario / spawners", timingStats.scenarioMsAvg, timingStats.scenarioMsMax);
                    showTimingRow("Physics update", timingStats.physicsMsAvg, timingStats.physicsMsMax);
                    showTimingRow("Collision update", timingStats.collisionMsAvg, timingStats.collisionMsMax);
                    showTimingRow("Network state generation", timingStats.netStateMsAvg, timingStats.netStateMsMax);
                    showTimingRow("Render snapshot capture", timingStats.renderSnapshotMsAvg, timingStats.renderSnapshotMsMax);
                    showTimingRow("Shared lock wait", timingStats.lockMsAvg, timingStats.lockMsMax);
                    showTimingRow("Outer simulation loop", timingStats.simLoopMsAvg, timingStats.simLoopMsMax);
                    ImGui::EndTable();
                }
                ImGui::TextDisabled("Timing average and max are published from the same rolling half-second window.");
                ImGui::Text("Snapshot tick: %llu", snap ? (unsigned long long)snap->simTickNumber : 0ull);
                ImGui::EndMenu();
            }

            ImGui::EndMainMenuBar();
            }
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
    if (netReceiveThread.joinable()) netReceiveThread.join();
    if (netSendThread.joinable()) netSendThread.join();
    if (networkReady)
        networkRuntime.net.Shutdown();
    gpuFlocking.Shutdown();

    return 0;
}
