/*
 * SandboxSceneOne.cpp — Milestone 1: Core Concurrency Threading Scaffold
 *
 * Architecture (Option B startup: load scene, then start threads):
 *
 *   Main / Render thread  (core 0) — GLFW, Vulkan, ImGui, camera control
 *   Simulation thread     (core 3) — ECS World, physics, collision, snapshot
 *   Networking thread     (core 1) — scaffold only; no real sockets yet
 *
 * The sim thread owns the live ECS World exclusively.  Every tick it produces a
 * WorldSnapshot (instances + lights + particles) which is published via a
 * mutex-protected shared_ptr buffer.  The render thread reads the latest
 * snapshot each frame and never touches the live World.
 *
 * Camera state lives entirely in the render thread; it is NOT part of the World
 * after the initial scene load extracts starting values.
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
#include <thread>
#include <atomic>
#include <memory>
#include <algorithm>
#include <cmath>
#include <string>
#include <mutex>
#include <cstdint>
#include "NetworkingSystem.h"
// ============================================================================
// Shared control state between threads
// ============================================================================
struct SimSharedState
{
    // Application lifetime
    std::atomic<bool> appRunning{ true };

    // Simulation controls (written by render/UI thread, read by sim thread)
    std::atomic<bool>  simRunning       { true };
    std::atomic<bool>  stepOnce         { false };
    std::atomic<bool>  useFixedTimestep { true };
    std::atomic<float> fixedDt          { 1.0f / 60.0f };
    std::atomic<int>   integratorType   { 0 };   // IntegratorType enum value
    std::atomic<bool> sendGlobalCommand{ false };

    // Scene switching: render thread writes, sim thread reads and acts
    std::atomic<int> requestedSceneIndex{ -1 };

    // Gravity enable flag: updated by sim thread after scene switch
    std::atomic<bool> gravityOn{ true };

    // Shared clear-colour (4 floats; written by UI, read by sim for snapshot)
    float clearColour[4] = { 0.1f, 0.1f, 0.1f, 1.0f };
    std::mutex clearColourMutex;

    // Target Hz — live-adjustable from UI
    std::atomic<float> targetRenderHz  { 60.0f };
    std::atomic<float> targetNetworkHz { 60.0f };
    std::atomic<float> targetSimHz     { 120.0f };

    // Measured Hz — written by each thread, read by UI
    std::atomic<float> measuredRenderHz  { 0.0f };
    std::atomic<float> measuredNetworkHz { 0.0f };
    std::atomic<float> measuredSimHz     { 0.0f };

    // Core indices actually assigned (for UI display)
    int renderCoreAssigned  = ThreadUtils::CORE_RENDER;
    int netCoreAssigned     = ThreadUtils::CORE_NET_0;
    int simCoreAssigned     = ThreadUtils::CORE_SIM_0;

    // Snapshot buffer
    SnapshotBuffer snapshotBuf;
};

// ============================================================================
// Camera state — owned exclusively by the render / UI thread
// ============================================================================
struct RenderCamera
{
    glm::vec3 position { 0.0f, 3.0f, -8.0f };
    glm::vec3 rotation { 0.0f };   // Euler (pitch, yaw, roll) in radians
    float fov      = 60.0f;
    float nearClip = 0.1f;
    float farClip  = 600.0f;
};

static CameraRenderData BuildCameraRenderData(const RenderCamera& cam, float aspect)
{
    CameraRenderData data{};
    data.position = cam.position;

    const float yaw   = cam.rotation.y;
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
// Snapshot builder — called by the sim thread after each tick
// ============================================================================
static std::shared_ptr<WorldSnapshot> CaptureSnapshot(
    World& world, uint64_t tickNumber, SimSharedState& shared)
{
    auto snap = std::make_shared<WorldSnapshot>();
    snap->simTickNumber = tickNumber;

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
        auto* tr  = world.getComponent<TransformComponent>(e);
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

        inst.meshName     = meshComp.meshName;
        inst.textureName  = meshComp.textureName;
        inst.shadingModel = mat->shadingModel;
        inst.diffuseColor = mat->diffuseColor;
        inst.specularColor = mat->specularColor;
        inst.shininess    = mat->shininess;
        inst.castsShadows    = mat->castsShadows;
        inst.receivesShadows = mat->receivesShadows;

        if (gForceShading == 0) inst.shadingModel = ShadingModel::Gouraud;
        if (gForceShading == 1) inst.shadingModel = ShadingModel::Phong;

        snap->instances.push_back(std::move(inst));
    });

    // Directional lights
    world.forEach<DirectionalLightComponent>([&](Entity, DirectionalLightComponent& light)
    {
        DirectionalLightRenderData d;
        d.direction = light.direction;
        d.color     = light.color;
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
        out.position  = tr->position;
        out.radius    = s.radius;
        out.color     = s.color;
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
    const float maxDt    = 0.25f;

    while (shared.appRunning.load(std::memory_order_relaxed))
    {
        // Live-apply target Hz changes
        lc.setTargetHz(shared.targetSimHz.load(std::memory_order_relaxed));

        float dt = lc.beginFrame();
        shared.measuredSimHz.store(lc.getMeasuredHz(), std::memory_order_relaxed);

        // Clamp to avoid runaway catch-up
        dt = std::min(dt, maxDt);

        // Read controls
        const bool   running       = shared.simRunning.load(std::memory_order_relaxed);
        const bool   useFixed      = shared.useFixedTimestep.load(std::memory_order_relaxed);
        const float  fdt           = shared.fixedDt.load(std::memory_order_relaxed);
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

        // ----- Handle scene switch request from render/UI thread -----
        {
            int desired = shared.requestedSceneIndex.load(std::memory_order_acquire);
            if (desired >= 0 && desired < scenarios.Count())
            {
                if (desired != scenarios.CurrentIndex())
                {
                    scenarios.SwitchTo(world, desired);
                    accumulator = 0.0f;  // discard stale accumulator

                    // Apply scene gravity
                    const bool grav = scenarios.Current()
                                          ? scenarios.Current()->GravityOn()
                                          : true;
                    physicsSystem.SetGravityEnabled(grav);
                    shared.gravityOn.store(grav, std::memory_order_relaxed);
                }
                // Only clear the value we consumed; if the render thread
                // has already posted a newer request, the CAS fails and
                // that request will be processed next tick.
                shared.requestedSceneIndex.compare_exchange_strong(
                    desired, -1,
                    std::memory_order_release,
                    std::memory_order_relaxed);
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
                // Variable timestep
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

        // Publish snapshot every tick
        shared.snapshotBuf.publish(CaptureSnapshot(world, tickNumber, shared));

        lc.endFrame();
    }
}

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

    // ---- Networking system ----
    NetworkingSystem net;
    if (!net.Init(cfg))
        return;

    LoopController lc(shared.targetNetworkHz.load(std::memory_order_relaxed));

    while (shared.appRunning.load(std::memory_order_relaxed))
    {
        lc.setTargetHz(shared.targetNetworkHz.load(std::memory_order_relaxed));

        float dt = lc.beginFrame();
        shared.measuredNetworkHz.store(lc.getMeasuredHz(), std::memory_order_relaxed);

        net.Update(dt);

        if (shared.sendGlobalCommand.exchange(false))
        {
            net.SendGlobalCommand(/* commandId */ 1);
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
    // ---- Option B: load scene single-threaded BEFORE starting threads ----
    World            world;
    PhysicsSystem    physicsSystem;
    CollisionSystem  collisionSystem;
    ScenarioManager  scenarios;

    scenarios.Add(std::make_unique<Scenario_PrimitiveScene>());
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>(
        "assets/scenes/newtonsCradle.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>(
        "assets/scenes/bouncingBalls.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>(
		"assets/scenes/piston.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>(
        "assets/scenes/sphereSpawners.bin"));
    scenarios.Add(std::make_unique<Scenario_FlatbufferScene>(
        "assets/scenes/tumbler.bin"));
    if (scenarios.Count() > 0)
        scenarios.SwitchTo(world, 0);

    // ---- Shared state — initialise Hz targets from config ----
    SimSharedState shared;
    shared.targetRenderHz .store(cfg.render_hz,     std::memory_order_relaxed);
    shared.targetNetworkHz.store(cfg.network_hz,    std::memory_order_relaxed);
    shared.targetSimHz    .store(cfg.simulation_hz, std::memory_order_relaxed);

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

    // ---- Extract initial camera state from the Overview camera entity ----
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
                renderCamera.fov      = cam->fov;
                renderCamera.nearClip = cam->nearClip;
                renderCamera.farClip  = cam->farClip;
            }
        }
    });

    // ---- Pin main (render) thread to core 0 ----
    const int numCores = ThreadUtils::LogicalCoreCount();
    const int renderCore = (ThreadUtils::CORE_RENDER < numCores)
                         ? ThreadUtils::CORE_RENDER : (numCores - 1);
    ThreadUtils::PinCurrentThreadToCore(renderCore);
    ThreadUtils::SetCurrentThreadName("Render");
    shared.renderCoreAssigned = renderCore;

    // ---- Produce an initial snapshot so the renderer doesn't start blank ----
    shared.snapshotBuf.publish(CaptureSnapshot(world, 0, shared));

    // ---- Start background threads ----
    std::thread simThread(SimulationThreadFunc,
        std::ref(shared), std::ref(world),
        std::ref(scenarios), std::ref(physicsSystem),
        std::ref(collisionSystem));

    std::thread netThread(NetworkingThreadFunc, std::ref(shared), std::cref(cfg));

    // ---- Render / UI loop state ----
    LoopController renderLC(shared.targetRenderHz.load(std::memory_order_relaxed));

    CameraRole activeCameraMode = CameraRole::Overview; // selects which preset to use
    const float maxFrameDt = 0.25f;

    // ImGui display copies (atomic reads are noisy inside ImGui calls)
    float displayRenderHz  = 0.0f;
    float displayNetHz     = 0.0f;
    float displaySimHz     = 0.0f;

    // Local copies of target Hz for ImGui sliders
    float uiRenderHz  = cfg.render_hz;
    float uiNetHz     = cfg.network_hz;
    float uiSimHz     = cfg.simulation_hz;

    // ---- Main render loop ----
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // Live-apply render Hz target
        renderLC.setTargetHz(shared.targetRenderHz.load(std::memory_order_relaxed));
        float dt = renderLC.beginFrame();
        shared.measuredRenderHz.store(renderLC.getMeasuredHz(), std::memory_order_relaxed);
        dt = std::min(dt, maxFrameDt);

        // ---- Camera movement (render thread only) ----
        if (!ImGui::GetIO().WantCaptureKeyboard)
        {
            const float lookSpeed = 1.5f;
            if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) renderCamera.rotation.y -= lookSpeed * dt;
            if (glfwGetKey(window, GLFW_KEY_LEFT)  == GLFW_PRESS) renderCamera.rotation.y += lookSpeed * dt;
            if (glfwGetKey(window, GLFW_KEY_UP)    == GLFW_PRESS) renderCamera.rotation.x += lookSpeed * dt;
            if (glfwGetKey(window, GLFW_KEY_DOWN)  == GLFW_PRESS) renderCamera.rotation.x -= lookSpeed * dt;

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
                renderCamera.position +=
                    (right * moveLocal.x + up * moveLocal.y + forward * moveLocal.z)
                    * (moveSpeed * dt);
            }
        }

        // ---- Snapshot: consume latest from sim thread ----
        std::shared_ptr<WorldSnapshot> snap = shared.snapshotBuf.consume();

        // ---- ImGui ----
        renderer.BeginImGuiFrame();

        // Refresh display values periodically
        displayRenderHz = shared.measuredRenderHz .load(std::memory_order_relaxed);
        displayNetHz    = shared.measuredNetworkHz.load(std::memory_order_relaxed);
        displaySimHz    = shared.measuredSimHz    .load(std::memory_order_relaxed);

        if (ImGui::BeginMainMenuBar())
        {
            // ---- Simulation menu ----
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

            // ---- View menu ----
            if (ImGui::BeginMenu("View"))
            {
                std::lock_guard<std::mutex> lk(shared.clearColourMutex);
                ImGui::ColorEdit3("Background", shared.clearColour);
                ImGui::EndMenu();
            }

            // ---- Camera menu ----
            if (ImGui::BeginMenu("Camera"))
            {
                bool isOverview   = (activeCameraMode == CameraRole::Overview);
                bool isNavigation = (activeCameraMode == CameraRole::Navigation);

                if (ImGui::MenuItem("Overview", nullptr, isOverview))
                {
                    activeCameraMode = CameraRole::Overview;
                    // Snap camera back to overview starting position
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
                ImGui::DragFloat3("Position",        &renderCamera.position.x, 0.1f);
                ImGui::DragFloat3("Rotation (rad)",  &renderCamera.rotation.x, 0.02f);
                ImGui::EndMenu();
            }

            // ---- Scene menu (Global UI: swap between loaded scenes) ----
            if (ImGui::BeginMenu("Scene"))
            {
                ImGui::SeparatorText("Switch Scene");
                ImGui::TextDisabled("(Global: all peers will switch)");
                ImGui::Separator();

                const int currentIdx = scenarios.CurrentIndex();
                for (int si = 0; si < scenarios.Count(); ++si)
                {
                    const bool selected = (si == currentIdx);
                    if (ImGui::MenuItem(scenarios.Get(si)->Name(),
                                        nullptr, selected))
                    {
                        if (si != currentIdx)
                            shared.requestedSceneIndex.store(
                                si, std::memory_order_release);
                    }
                }

                ImGui::Separator();
                const bool grav = shared.gravityOn.load(std::memory_order_relaxed);
                ImGui::Text("Gravity: %s", grav ? "ON" : "OFF");

                ImGui::EndMenu();
            }

            // ---- Concurrency menu ----
            if (ImGui::BeginMenu("Concurrency"))
            {
                ImGui::SeparatorText("Target Frequencies");

                if (ImGui::SliderFloat("Render Hz",  &uiRenderHz,  0.0f, 300.0f, "%.0f"))
                    shared.targetRenderHz.store(uiRenderHz,  std::memory_order_relaxed);
                if (ImGui::SliderFloat("Network Hz", &uiNetHz,     0.0f, 120.0f, "%.0f"))
                    shared.targetNetworkHz.store(uiNetHz,   std::memory_order_relaxed);
                if (ImGui::SliderFloat("Sim Hz",     &uiSimHz,     0.0f, 500.0f, "%.0f"))
                    shared.targetSimHz.store(uiSimHz,       std::memory_order_relaxed);
                if (ImGui::Button("Send Global Command"))
                {
                    shared.sendGlobalCommand.store(true, std::memory_order_relaxed);
                }

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

        // ---- Build RenderScene from snapshot + camera and render ----
        if (snap)
        {
            RenderScene scene{};
            scene.camera           = BuildCameraRenderData(renderCamera, renderer.getAspectRatio());
            scene.ambientLight     = snap->ambientLight;
            scene.clearColor       = snap->clearColor;
            scene.instances        = snap->instances;
            scene.particles        = snap->particles;
            scene.directionalLights = snap->directionalLights;
            scene.sparkLights      = snap->sparkLights;
            renderer.render(scene);
        }

        renderLC.endFrame();
    }

    // ---- Shutdown: signal threads and join ----
    shared.appRunning.store(false, std::memory_order_relaxed);

    if (simThread.joinable()) simThread.join();
    if (netThread.joinable()) netThread.join();

    return 0;
}
