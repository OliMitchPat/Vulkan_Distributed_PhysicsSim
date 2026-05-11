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

    // Outgoing GLOBAL UI requests (render thread -> networking thread)
    std::atomic<bool> sendSceneChange{ false };
    std::atomic<int>  sendSceneIndex{ -1 };

    std::atomic<bool> sendGravityChange{ false };
    std::atomic<bool> sendGravityEnabled{ true };

    // Incoming GLOBAL UI commands (networking thread -> sim thread)
    std::atomic<bool> pendingSceneChange{ false };
    std::atomic<int>  pendingSceneIndex{ -1 };

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
    std::unordered_map<uint32_t, ReplicaState> inReplicaLatest; // objectId -> latest

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

            RenderInstance inst{};
            glm::mat4 M{ 1.0f };
            M = glm::translate(M, tr->position);
            M = glm::rotate(M, tr->rotation.y, glm::vec3(0, 1, 0));
            M = glm::rotate(M, tr->rotation.x, glm::vec3(1, 0, 0));
            M = glm::rotate(M, tr->rotation.z, glm::vec3(0, 0, 1));
            M = glm::scale(M, tr->scale);
            inst.model = M;

            inst.meshName = meshComp.meshName;
            inst.textureName = meshComp.textureName;
            inst.shadingModel = mat->shadingModel;
            inst.diffuseColor = mat->diffuseColor;
            inst.specularColor = mat->specularColor;
            inst.shininess = mat->shininess;
            inst.castsShadows = mat->castsShadows;
            inst.receivesShadows = mat->receivesShadows;

            // Local UI override: colour by owner
            if (displayMode == 1)
            {
                if (auto* owner = world.getComponent<OwnerComponent>(e))
                {
                    inst.diffuseColor = OwnerColor(owner->ownerId);
                    inst.specularColor = glm::vec3(0.05f);
                    inst.shininess = 4.0f;
                }
            }

            if (gForceShading == 0) inst.shadingModel = ShadingModel::Gouraud;
            if (gForceShading == 1) inst.shadingModel = ShadingModel::Phong;

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
    CollisionSystem& collisionSystem)
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

                    // Keep current GLOBAL gravity policy across scene switches
                    const bool grav = shared.gravityOn.load(std::memory_order_relaxed);
                    physicsSystem.SetGravityEnabled(grav);

                    // IMPORTANT: clear replication state so old-scene object IDs/states
                    // don't bleed into the newly loaded scene.
                    {
                        std::lock_guard<std::mutex> lk(shared.inSnapMutex);
                        shared.inReplicaLatest.clear();
                    }
                    {
                        std::lock_guard<std::mutex> lk(shared.outSnapMutex);
                        shared.outOwnedItems.clear();
                        shared.outTick = 0;
                        shared.outDirty = false;
                    }
                }

                shared.requestedSceneIndex.compare_exchange_strong(
                    desired, -1,
                    std::memory_order_release,
                    std::memory_order_relaxed);
            }
        }

        // ----- Milestone 5: apply incoming replica snapshots (net -> sim) -----
        {
            std::unordered_map<uint32_t, ReplicaState> latest;
            {
                std::lock_guard<std::mutex> lk(shared.inSnapMutex);
                latest = shared.inReplicaLatest; // copy to minimize lock time
            }

            for (auto& kv : latest)
            {
                const uint32_t id = kv.first;
                const ReplicaState& st = kv.second;

                Entity e = (Entity)id;

                auto* phys = world.getComponent<PhysicsComponent>(e);
                auto* tr = world.getComponent<TransformComponent>(e);
                if (!phys || !tr) continue;

                // Don't override authoritative bodies
                if (phys->body.IsDynamic()) continue;

                phys->body.SetPosition(st.pos);
                phys->body.SetOrientation(st.rot);
                phys->body.SetLinearVelocity(st.linVel);
                phys->body.SetAngularVelocity(st.angVel);

                tr->position = st.pos;
                tr->rotation = glm::eulerAngles(st.rot);
            }
        }

        if (Scenario* s = scenarios.Current())
        {
            if (useFixed)
            {
                if (running)
                    accumulator += dt;
                else if (doStep)
                    accumulator += fdt;

                while (accumulator >= fdt)
                {
                    s->Update(world, fdt);
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
                    float stepDt = running ? dt : fdt;
                    s->Update(world, stepDt);
                    physicsSystem.Update(world, stepDt);
                    collisionSystem.Update(world);
                    ++tickNumber;
                }
            }
        }

        // ----- Milestone 5: build outgoing STATE_SNAPSHOT (owned dynamics) -----
        {
            std::vector<Net::StateSnapshotItem> items;
            items.reserve(256);

            world.forEach<PhysicsComponent>([&](Entity e, PhysicsComponent& phys)
                {
                    if (!phys.body.IsDynamic())
                        return;

                    Net::StateSnapshotItem it{};
                    it.objectId = (uint32_t)e;

                    const glm::vec3 p = phys.body.Position();
                    const glm::quat q = phys.body.Orientation();
                    const glm::vec3 v = phys.body.LinearVelocity();
                    const glm::vec3 w = phys.body.AngularVelocity();

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

    // Snapshot send throttling (prevents UDP flood starving GLOBAL commands)
    float snapshotSendAccum = 0.0f;
    constexpr float SNAPSHOT_SEND_HZ = 20.0f;                 // tune: 15–30 is usually fine
    constexpr float SNAPSHOT_SEND_INTERVAL = 1.0f / SNAPSHOT_SEND_HZ;

    while (shared.appRunning.load(std::memory_order_relaxed))
    {
        lc.setTargetHz(shared.targetNetworkHz.load(std::memory_order_relaxed));

        float dt = lc.beginFrame();
        shared.measuredNetworkHz.store(lc.getMeasuredHz(), std::memory_order_relaxed);

        net.Update(dt);

        // ---- Send outgoing global commands (from render thread) ----
        if (shared.sendSceneChange.exchange(false, std::memory_order_acq_rel))
        {
            const int idx = shared.sendSceneIndex.load(std::memory_order_relaxed);
            net.SendSceneChange(idx);
        }

        if (shared.sendGravityChange.exchange(false, std::memory_order_acq_rel))
        {
            const bool enabled = shared.sendGravityEnabled.load(std::memory_order_relaxed);
            net.SendGravityEnabled(enabled);
        }

        // ---- Receive incoming global commands and forward to sim thread ----
        GlobalCommandPayload cmd{};
        while (net.PopReceivedGlobalCommand(cmd))
        {
            const auto type = (GlobalCommandType)cmd.commandType;

            const ULONGLONG t = GetTickCount64();
            std::cout << "[T] NET popped GLOBAL cmd scene=" << cmd.sceneIndex
                << " t=" << t << "ms\n";

            if (type == GlobalCommandType::SceneChange)
            {
                shared.pendingSceneIndex.store(cmd.sceneIndex, std::memory_order_relaxed);
                shared.pendingSceneChange.store(true, std::memory_order_release);
            }
            else if (type == GlobalCommandType::GravityOnOff)
            {
                shared.pendingGravityEnabled.store(cmd.gravityEnabled != 0, std::memory_order_relaxed);
                shared.pendingGravityChange.store(true, std::memory_order_release);
            }
        }

        // ---- Milestone 5: send STATE_SNAPSHOT (throttled) ----
        snapshotSendAccum += dt;

        if (snapshotSendAccum >= SNAPSHOT_SEND_INTERVAL)
        {
            // keep leftover time (more stable than setting to 0)
            snapshotSendAccum -= SNAPSHOT_SEND_INTERVAL;

            std::vector<StateSnapshotItem> items;
            uint32_t tick = 0;
            bool dirty = false;

            {
                std::lock_guard<std::mutex> lk(shared.outSnapMutex);
                dirty = shared.outDirty;
                if (dirty)
                {
                    items = shared.outOwnedItems; // copy latest
                    tick = shared.outTick;
                    shared.outDirty = false;
                }
            }

            if (dirty && !items.empty())
            {
                net.SendStateSnapshot(tick, items.data(), (uint16_t)items.size());
            }
        }

        // ---- Milestone 5: receive STATE_SNAPSHOT and store latest per object ----
        {
            std::vector<StateSnapshotItem> items;
            uint32_t tick = 0;

            while (net.PopReceivedStateSnapshot(items, tick))
            {
                std::lock_guard<std::mutex> lk(shared.inSnapMutex);

                for (const auto& it : items)
                {
                    ReplicaState st{};
                    st.objectId = it.objectId;
                    st.pos = glm::vec3(it.pos.x, it.pos.y, it.pos.z);
                    st.rot = glm::quat(it.rot.w, it.rot.x, it.rot.y, it.rot.z); // glm quat is (w,x,y,z)
                    st.linVel = glm::vec3(it.linVel.x, it.linVel.y, it.linVel.z);
                    st.angVel = glm::vec3(it.angVel.x, it.angVel.y, it.angVel.z);
                    st.tick = tick;

                    shared.inReplicaLatest[it.objectId] = st;
                }
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
    World            world;
    PhysicsSystem    physicsSystem;
    physicsSystem.SetLocalPeerId(cfg.peer_id);

    CollisionSystem  collisionSystem;
    ScenarioManager  scenarios;

    scenarios.Add(std::make_unique<Scenario_PrimitiveScene>());
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/newtonsCradle.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/bouncingBalls.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/piston.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/sphereSpawners.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>("assets/scenes/tumbler.bin"));

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
    std::thread simThread(SimulationThreadFunc,
        std::ref(shared), std::ref(world),
        std::ref(scenarios), std::ref(physicsSystem),
        std::ref(collisionSystem));

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
                            std::cout << "[T] UI scene click idx=" << si
                                << " t=" << t << "ms\n";

                            // Apply locally immediately
                            shared.requestedSceneIndex.store(si, std::memory_order_release);

                            // Send globally
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