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
#include <sstream>
#include <limits>

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

struct RuntimeDebugCounts
{
    uint32_t localOwned = 0;
    uint32_t remoteOwned = 0;
    uint32_t staticObjects = 0;
    uint32_t animatedObjects = 0;
    uint32_t spawnedObjects = 0;
    uint32_t dynamicBodies = 0;
    uint32_t collisionCandidates = 0;
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
    std::atomic<float> measuredNetworkSendHz{ 0.0f };
    std::atomic<float> measuredNetworkRecvHz{ 0.0f };
    std::atomic<float> measuredSimHz{ 0.0f };
    std::atomic<float> measuredSimLoopHz{ 0.0f };
    std::mutex netStatsMutex;
    Net::NetworkStats netStats{};
    std::mutex simTimingMutex;
    SimTimingStats simTiming{};
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
// Camera state — owned exclusively by the render / UI thread
// ============================================================================
struct RenderCamera
{
    glm::vec3 position{ 0.0f, 3.0f, -8.0f };
    glm::vec3 rotation{ 0.0f };   // Euler (pitch, yaw, roll) in radians
    glm::quat orientation{ 1.0f, 0.0f, 0.0f, 0.0f };
    bool useOrientation = false;
    float localForwardZ = -1.0f;
    CameraComponent::Projection projection = CameraComponent::Projection::Perspective;
    float fov = 60.0f;
    float orthoSize = 10.0f;
    float nearClip = 0.1f;
    float farClip = 600.0f;
};

static CameraRenderData BuildCameraRenderData(const RenderCamera& cam, float aspect)
{
    CameraRenderData data{};
    data.position = cam.position;

    glm::vec3 forward{};
    glm::vec3 up{ 0, 1, 0 };
    if (cam.useOrientation)
    {
        const glm::mat3 basis = glm::mat3_cast(cam.orientation);
        forward = glm::normalize(basis * glm::vec3(0, 0, cam.localForwardZ));
        up = glm::normalize(basis * glm::vec3(0, 1, 0));
    }
    else
    {
        const float yaw = cam.rotation.y;
        const float pitch = cam.rotation.x;
        forward = glm::vec3{
            std::cos(pitch) * std::sin(yaw),
            std::sin(pitch),
            std::cos(pitch) * std::cos(yaw)
        };
    }

    data.view = glm::lookAt(cam.position, cam.position + forward, up);

    if (cam.projection == CameraComponent::Projection::Orthographic)
    {
        const float halfHeight = std::max(0.1f, cam.orthoSize) * 0.5f;
        const float halfWidth = halfHeight * aspect;
        data.proj = glm::ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, cam.nearClip, cam.farClip);
    }
    else
    {
        const float fovRad = glm::radians(cam.fov);
        data.proj = glm::perspective(fovRad, aspect, cam.nearClip, cam.farClip);
    }
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

struct SceneCameraOption
{
    Entity entity = INVALID_ENTITY;
    std::string name;
    CameraComponent::Projection projection = CameraComponent::Projection::Perspective;
};

static std::vector<SceneCameraOption> CollectSceneCameras(World& world)
{
    std::vector<SceneCameraOption> cameras;
    world.forEach<CameraComponent>([&](Entity e, CameraComponent& cam)
        {
            SceneCameraOption option{};
            option.entity = e;
            option.projection = cam.projection;
            if (auto* name = world.getComponent<NameComponent>(e))
                option.name = name->name;
            if (option.name.empty())
                option.name = "Camera " + std::to_string(cameras.size() + 1);
            cameras.push_back(std::move(option));
        });
    return cameras;
}

static glm::vec3 EstimateSceneCenter(World& world)
{
    glm::vec3 sum{ 0.0f };
    uint32_t count = 0;

    world.forEach<RenderMeshComponent>([&](Entity e, RenderMeshComponent&)
        {
            if (world.getComponent<CameraComponent>(e))
                return;
            if (auto* tr = world.getComponent<TransformComponent>(e))
            {
                sum += tr->position;
                ++count;
            }
        });

    if (count == 0)
        return glm::vec3(0.0f);

    return sum / static_cast<float>(count);
}

struct SceneBounds
{
    glm::vec3 min{ 0.0f };
    glm::vec3 max{ 0.0f };
    bool valid = false;
};

static SceneBounds EstimateSceneBounds(World& world)
{
    SceneBounds bounds{};

    world.forEach<RenderMeshComponent>([&](Entity e, RenderMeshComponent&)
        {
            if (world.getComponent<CameraComponent>(e))
                return;

            const auto* tr = world.getComponent<TransformComponent>(e);
            if (!tr)
                return;

            const glm::vec3 halfExtent = glm::max(glm::abs(tr->scale) * 0.5f, glm::vec3(0.5f));
            const glm::vec3 pMin = tr->position - halfExtent;
            const glm::vec3 pMax = tr->position + halfExtent;

            if (!bounds.valid)
            {
                bounds.min = pMin;
                bounds.max = pMax;
                bounds.valid = true;
                return;
            }

            bounds.min = glm::min(bounds.min, pMin);
            bounds.max = glm::max(bounds.max, pMax);
        });

    return bounds;
}

static void FitOrthographicCameraToScene(World& world, RenderCamera& renderCamera)
{
    const SceneBounds bounds = EstimateSceneBounds(world);
    if (!bounds.valid)
        return;

    const glm::mat3 basis = glm::mat3_cast(renderCamera.orientation);
    const glm::vec3 right = glm::normalize(basis * glm::vec3(1, 0, 0));
    const glm::vec3 up = glm::normalize(basis * glm::vec3(0, 1, 0));
    const glm::vec3 forward = glm::normalize(basis * glm::vec3(0, 0, renderCamera.localForwardZ));
    const glm::vec3 center = (bounds.min + bounds.max) * 0.5f;
    const float authoredDistance = glm::dot(center - renderCamera.position, forward);
    const float viewDistance = std::max(1.0f, authoredDistance);

    // Orthographic cameras should frame around their view centre. Some authored
    // top/side cameras use a good direction but are slightly off-centre, which
    // leaves the scene outside the orthographic box.
    renderCamera.position = center - forward * viewDistance;

    float minRight = std::numeric_limits<float>::max();
    float maxRight = std::numeric_limits<float>::lowest();
    float minUp = std::numeric_limits<float>::max();
    float maxUp = std::numeric_limits<float>::lowest();
    float minDepth = std::numeric_limits<float>::max();
    float maxDepth = std::numeric_limits<float>::lowest();

    for (int xi = 0; xi < 2; ++xi)
    {
        for (int yi = 0; yi < 2; ++yi)
        {
            for (int zi = 0; zi < 2; ++zi)
            {
                const glm::vec3 p{
                    xi ? bounds.max.x : bounds.min.x,
                    yi ? bounds.max.y : bounds.min.y,
                    zi ? bounds.max.z : bounds.min.z
                };
                const glm::vec3 rel = p - renderCamera.position;
                const float r = glm::dot(rel, right);
                const float u = glm::dot(rel, up);
                const float d = glm::dot(rel, forward);

                minRight = std::min(minRight, r);
                maxRight = std::max(maxRight, r);
                minUp = std::min(minUp, u);
                maxUp = std::max(maxUp, u);
                minDepth = std::min(minDepth, d);
                maxDepth = std::max(maxDepth, d);
            }
        }
    }

    const float width = std::max(0.1f, maxRight - minRight);
    const float height = std::max(0.1f, maxUp - minUp);
    const float fittedSize = std::max(width, height) * 1.15f;
    renderCamera.orthoSize = std::max(renderCamera.orthoSize, fittedSize);

    const float depthPad = std::max(2.0f, (maxDepth - minDepth) * 0.25f);
    renderCamera.nearClip = std::max(0.01f, std::min(0.1f, minDepth - depthPad));
    renderCamera.farClip = std::max(renderCamera.nearClip + 1.0f, maxDepth + depthPad);
}

static bool ApplySceneCamera(World& world, Entity cameraEntity, RenderCamera& renderCamera)
{
    auto* tr = world.getComponent<TransformComponent>(cameraEntity);
    auto* cam = world.getComponent<CameraComponent>(cameraEntity);
    if (!tr || !cam)
        return false;

    renderCamera.position = tr->position;
    renderCamera.rotation = tr->rotation;
    renderCamera.orientation = tr->orientation;
    renderCamera.useOrientation = true;
    {
        const glm::mat3 basis = glm::mat3_cast(renderCamera.orientation);
        const glm::vec3 toScene = EstimateSceneCenter(world) - renderCamera.position;
        if (glm::dot(toScene, toScene) > 1e-6f)
        {
            const glm::vec3 forwardNegZ = glm::normalize(basis * glm::vec3(0, 0, -1));
            const glm::vec3 forwardPosZ = glm::normalize(basis * glm::vec3(0, 0, 1));
            renderCamera.localForwardZ =
                glm::dot(glm::normalize(toScene), forwardPosZ) >
                glm::dot(glm::normalize(toScene), forwardNegZ)
                ? 1.0f
                : -1.0f;
        }
        else
        {
            renderCamera.localForwardZ = -1.0f;
        }
    }
    renderCamera.projection = cam->projection;
    renderCamera.fov = cam->fov;
    renderCamera.orthoSize = cam->orthoSize;
    renderCamera.nearClip = cam->nearClip;
    renderCamera.farClip = cam->farClip;
    if (renderCamera.projection == CameraComponent::Projection::Orthographic)
        FitOrthographicCameraToScene(world, renderCamera);
    return true;
}

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
    counts.collisionContacts = collisionSystem.LastContactCount();
    return counts;
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
    const float maxDt = 0.25f;

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
                    physicsSystem.InitializePhysicsBodies(world);
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
                if (!smoothingEnabled)
                {
                    phys->body.SetPosition(target.pos);
                    phys->body.SetOrientation(target.rot);
                    phys->body.SetLinearVelocity(target.linVel);
                    phys->body.SetAngularVelocity(target.angVel);
                    tr->position = target.pos;
                    tr->rotation = glm::eulerAngles(target.rot);
                    continue;
                }

                const glm::vec3 currentPos = phys->body.Position();
                const glm::quat currentRot = phys->body.Orientation();
                const glm::vec3 currentLinVel = phys->body.LinearVelocity();
                const glm::vec3 currentAngVel = phys->body.AngularVelocity();

                const float posError = glm::length(target.pos - currentPos);
                const bool largeError = posError > correctionThreshold;

                float spring = (largeError ? 7.0f : 16.0f) * std::max(0.0f, correctionStrength);
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

                constexpr int MAX_FIXED_SUBSTEPS_PER_FRAME = 32;
                int substepsThisFrame = 0;

                while (accumulator >= fdt && substepsThisFrame < MAX_FIXED_SUBSTEPS_PER_FRAME)
                {
                    size_t pendingSpawnsBeforeUpdate = 0;
                    if (auto* fb = dynamic_cast<Scenario_FlatbufferScene*>(s))
                        pendingSpawnsBeforeUpdate = fb->PendingSpawnCount();

                    const double scenarioStartSec = NowSeconds();
                    s->Update(world, fdt, currentSceneGeneration);
                    const double physicsStartSec = NowSeconds();

                    if (auto* fb = dynamic_cast<Scenario_FlatbufferScene*>(s))
                    {
                        if (fb->PendingSpawnCount() != pendingSpawnsBeforeUpdate)
                            physicsSystem.InitializePhysicsBodies(world);
                    }

                    physicsSystem.Update(world, fdt);
                    const double collisionStartSec = NowSeconds();
                    collisionSystem.Update(world);
                    const double collisionEndSec = NowSeconds();

                    timing.scenario.Add(TimingAccumulator::Ms(scenarioStartSec, physicsStartSec));
                    timing.physics.Add(TimingAccumulator::Ms(physicsStartSec, collisionStartSec));
            timing.collision.Add(TimingAccumulator::Ms(collisionStartSec, collisionEndSec));

                    accumulator -= fdt;
                    ++tickNumber;
                    ++substepsThisFrame;
                    ++simTicksThisLoop;
                }

                if (accumulator >= fdt)
                    accumulator = 0.0f;
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
                    const double physicsStartSec = NowSeconds();

                    if (auto* fb = dynamic_cast<Scenario_FlatbufferScene*>(s))
                    {
                        if (fb->PendingSpawnCount() != pendingSpawnsBeforeUpdate)
                            physicsSystem.InitializePhysicsBodies(world);
                    }

                    physicsSystem.Update(world, stepDt);
                    const double collisionStartSec = NowSeconds();
                    collisionSystem.Update(world);
                    const double collisionEndSec = NowSeconds();

                    timing.scenario.Add(TimingAccumulator::Ms(scenarioStartSec, physicsStartSec));
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
            shared.snapshotBuf.publish(CaptureSnapshot(world, tickNumber, shared));
            const double renderSnapshotEndSec = NowSeconds();
            timing.renderSnapshot.Add(TimingAccumulator::Ms(renderSnapshotStartSec, renderSnapshotEndSec));
        }

        timing.simLoop.Add(TimingAccumulator::Ms(simLoopStartSec, NowSeconds()));

        SimTimingStats publishedTiming{};
        if (timing.PublishIfReady(NowSeconds(), publishedTiming))
        {
            std::lock_guard<std::mutex> lk(shared.simTimingMutex);
            shared.simTiming = publishedTiming;
        }

        {
            RuntimeDebugCounts counts =
                BuildRuntimeDebugCounts(world, localPeerId, physicsSystem, collisionSystem);
            std::lock_guard<std::mutex> lk(shared.debugCountsMutex);
            shared.debugCounts = counts;
        }

        lc.endFrame();
    }
}

struct NetworkRuntime
{
    Net::NetworkingSystem net;
    std::mutex mutex;
};

static int ClampCoreForMachine(int desiredCore, int fallbackCore)
{
    const int numCores = ThreadUtils::LogicalCoreCount();
    if (numCores <= 0)
        return 0;
    if (desiredCore < numCores)
        return desiredCore;
    if (fallbackCore < numCores)
        return fallbackCore;
    return numCores - 1;
}

// ============================================================================
// Network receive thread. Pinned to logical processor 1 where available.
// ============================================================================
static void NetworkReceiveThreadFunc(
    SimSharedState& shared,
    const Net::PeerConfig& cfg,
    NetworkRuntime& runtime)
{
    using namespace Net;

    const int assignedRecvCore =
        ClampCoreForMachine(ThreadUtils::CORE_NET_0, ThreadUtils::CORE_RENDER);
    ThreadUtils::PinCurrentThread(ThreadUtils::CoreMask(assignedRecvCore));
    ThreadUtils::SetCurrentThreadName("NetRecv");
    shared.netRecvCoreAssigned = assignedRecvCore;
    shared.netRecvThreadRunning.store(true, std::memory_order_release);

    LoopController lc(shared.targetNetworkHz.load(std::memory_order_relaxed));

    while (shared.appRunning.load(std::memory_order_relaxed))
    {
        lc.setTargetHz(shared.targetNetworkHz.load(std::memory_order_relaxed));
        const float dt = lc.beginFrame();
        shared.measuredNetworkRecvHz.store(lc.getMeasuredHz(), std::memory_order_relaxed);

        std::vector<GlobalCommandPayload> commands;
        std::vector<std::pair<uint32_t, std::vector<StateSnapshotItem>>> snapshots;
        std::vector<SpawnObjectPayload> spawns;
        NetworkStats stats{};

        {
            std::lock_guard<std::mutex> netLock(runtime.mutex);
            runtime.net.SetCurrentSceneGeneration(
                shared.sceneGeneration.load(std::memory_order_acquire));
            runtime.net.UpdateReceive(dt);

            GlobalCommandPayload cmd{};
            while (runtime.net.PopReceivedGlobalCommand(cmd))
                commands.push_back(cmd);

            std::vector<StateSnapshotItem> items;
            uint32_t tick = 0;
            while (runtime.net.PopReceivedStateSnapshot(items, tick))
            {
                snapshots.emplace_back(tick, std::move(items));
                items.clear();
            }

            SpawnObjectPayload payload{};
            while (runtime.net.PopReceivedSpawnObject(payload))
                spawns.push_back(payload);

            stats = runtime.net.GetStats();
        }

        {
            std::lock_guard<std::mutex> lk(shared.netStatsMutex);
            shared.netStats = stats;
        }

        for (const auto& cmd : commands)
        {
            const auto type = (GlobalCommandType)cmd.commandType;

            const ULONGLONG t = GetTickCount64();
            std::cout << "[T] NET recv popped GLOBAL cmd scene=" << cmd.sceneIndex
                << " generation=" << cmd.sceneGeneration
                << " t=" << t << "ms\n";

            if (type == GlobalCommandType::SceneChange)
            {
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

                {
                    std::lock_guard<std::mutex> netLock(runtime.mutex);
                    runtime.net.SetCurrentSceneGeneration(cmd.sceneGeneration);
                    runtime.net.ClearSceneObjectTraffic();
                    runtime.net.PauseSnapshotTraffic(0.25f);
                }

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

        for (auto& snapshot : snapshots)
        {
            const uint32_t tick = snapshot.first;
            const double nowSec = NowSeconds();

            std::lock_guard<std::mutex> lk(shared.inSnapMutex);
            for (const auto& it : snapshot.second)
            {
                auto latestIt = shared.inReplicaLatestTick.find(it.objectId);
                if (latestIt != shared.inReplicaLatestTick.end() && tick <= latestIt->second)
                {
                    shared.staleReplicaPacketsIgnored.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                shared.inReplicaLatestTick[it.objectId] = tick;

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

        if (!spawns.empty())
        {
            const uint32_t currentGeneration =
                shared.sceneGeneration.load(std::memory_order_acquire);

            std::vector<SpawnObjectPayload> validSpawns;
            for (const auto& payload : spawns)
            {
                if (payload.sceneGeneration == currentGeneration)
                    validSpawns.push_back(payload);
            }

            if (!validSpawns.empty())
            {
                std::lock_guard<std::mutex> lk(shared.inSpawnMutex);
                shared.inSpawnEvents.insert(
                    shared.inSpawnEvents.end(),
                    validSpawns.begin(),
                    validSpawns.end());
            }
        }

        lc.endFrame();
    }

    shared.netRecvThreadRunning.store(false, std::memory_order_release);
}

// ============================================================================
// Network send thread. Pinned to logical processor 2 where available.
// ============================================================================
static void NetworkSendThreadFunc(
    SimSharedState& shared,
    const Net::PeerConfig& cfg,
    NetworkRuntime& runtime)
{
    using namespace Net;

    const int assignedSendCore =
        ClampCoreForMachine(ThreadUtils::CORE_NET_1, ThreadUtils::CORE_NET_0);
    ThreadUtils::PinCurrentThread(ThreadUtils::CoreMask(assignedSendCore));
    ThreadUtils::SetCurrentThreadName("NetSend");
    shared.netSendCoreAssigned = assignedSendCore;
    shared.netSendThreadRunning.store(true, std::memory_order_release);

    LoopController lc(shared.targetNetworkHz.load(std::memory_order_relaxed));

    // Snapshot send throttling. 0 Hz means every fresh simulation tick.
    float snapshotSendAccum = 0.0f;

    while (shared.appRunning.load(std::memory_order_relaxed))
    {
        lc.setTargetHz(shared.targetNetworkHz.load(std::memory_order_relaxed));

        float dt = lc.beginFrame();
        shared.measuredNetworkSendHz.store(lc.getMeasuredHz(), std::memory_order_relaxed);
        shared.measuredNetworkHz.store(lc.getMeasuredHz(), std::memory_order_relaxed);

        // Keep networking layer aligned with the currently valid scene generation.
        Net::SnapshotImpairmentSettings impair{};
        impair.enabled = shared.netImpairmentEnabled.load(std::memory_order_relaxed);
        impair.latencyMs = shared.netLatencyMs.load(std::memory_order_relaxed);
        impair.jitterMs = shared.netJitterMs.load(std::memory_order_relaxed);
        impair.dropPercent = shared.netDropPercent.load(std::memory_order_relaxed);
        {
            std::lock_guard<std::mutex> netLock(runtime.mutex);
            runtime.net.SetCurrentSceneGeneration(
                shared.sceneGeneration.load(std::memory_order_acquire));
            runtime.net.SetSnapshotImpairment(impair);
        }

        // ------------------------------------------------------------
        // High-priority outgoing global commands.
        // These must happen before reliable resends, so they are not stuck
        // behind snapshot/resend processing.
        // ------------------------------------------------------------

        if (shared.sendSceneChange.exchange(false, std::memory_order_acq_rel))
        {
            const int idx =
                shared.sendSceneIndex.load(std::memory_order_relaxed);

            const uint32_t generation =
                shared.sendSceneGeneration.load(std::memory_order_acquire);

            {
                std::lock_guard<std::mutex> netLock(runtime.mutex);
                // Old scene object traffic is no longer valid.
                runtime.net.ClearSceneObjectTraffic();
                runtime.net.PauseSnapshotTraffic(0.25f);
                runtime.net.SetCurrentSceneGeneration(generation);
                runtime.net.SendSceneChange(idx, generation);
            }
        }

        if (shared.sendGravityChange.exchange(false, std::memory_order_acq_rel))
        {
            const bool enabled =
                shared.sendGravityEnabled.load(std::memory_order_relaxed);

            {
                std::lock_guard<std::mutex> netLock(runtime.mutex);
                runtime.net.PauseSnapshotTraffic(0.10f);
                runtime.net.SendGravityEnabled(enabled);
            }
        }

        // ------------------------------------------------------------
        // Process reliable resends / delayed outgoing snapshots.
        // ------------------------------------------------------------

        NetworkStats stats{};
        {
            std::lock_guard<std::mutex> netLock(runtime.mutex);
            runtime.net.UpdateSend(dt);
            stats = runtime.net.GetStats();
        }

        {
            std::lock_guard<std::mutex> lk(shared.netStatsMutex);
            shared.netStats = stats;
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

                std::lock_guard<std::mutex> netLock(runtime.mutex);
                runtime.net.SendSpawnObject(spawn);
            }
        }

        // ------------------------------------------------------------
        // Send STATE_SNAPSHOT packets.
        // ------------------------------------------------------------

        snapshotSendAccum += dt;
        const float snapshotHz = shared.snapshotSendHz.load(std::memory_order_relaxed);
        const float snapshotInterval = (snapshotHz > 0.0f) ? (1.0f / snapshotHz) : 0.0f;

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
        else if (snapshotHz <= 0.0f || snapshotSendAccum >= snapshotInterval)
        {
            if (snapshotInterval > 0.0f)
                snapshotSendAccum = std::max(0.0f, snapshotSendAccum - snapshotInterval);
            else
                snapshotSendAccum = 0.0f;

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

                std::lock_guard<std::mutex> netLock(runtime.mutex);
                runtime.net.SendStateSnapshot(
                    tick,
                    generation,
                    items.data(),
                    (uint32_t)items.size());
            }
        }

        lc.endFrame();
    }

    shared.netSendThreadRunning.store(false, std::memory_order_release);
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
                    renderCamera.orientation =
                        glm::normalize(glm::angleAxis(pitchDelta, right) * renderCamera.orientation);
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
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) moveLocal.x -= 1.0f;
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) moveLocal.x += 1.0f;
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
                    right = glm::normalize(basis * glm::vec3(1, 0, 0));
                }
                else
                {
                    float yaw = renderCamera.rotation.y;
                    forward = glm::vec3(std::sin(yaw), 0.0f, std::cos(yaw));
                    right = glm::vec3(forward.z, 0.0f, -forward.x);
                }
                glm::vec3 up(0.0f, 1.0f, 0.0f);
                renderCamera.position += (right * moveLocal.x + up * moveLocal.y + forward * moveLocal.z) * (moveSpeed * dt);
            }
        }

        // Snapshot: consume latest from sim thread
        std::shared_ptr<WorldSnapshot> snap = shared.snapshotBuf.consume();

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
                            shared.sceneTransitionRemainingSec.store(0.10f, std::memory_order_release);
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

    return 0;
}
