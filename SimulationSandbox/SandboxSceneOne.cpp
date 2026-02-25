#include "World.h"
#include "ScenarioManager.h"
#include "Scenario.h"
#include "RenderSystem.h"
#include "Renderer.h"
#include "imGui.h"
#include <GLFW/glfw3.h>
#include "Scenario_PrimitiveScene.h"

static float GetDeltaTimeSeconds()
{
    static double last = glfwGetTime();
    double now = glfwGetTime();
    double dt = now - last;
    last = now;
    return (float)dt;
}

// Return: 0 = exit, 1 = reset (optional), anything else you want
int RunSandbox(GLFWwindow* window, Renderer& renderer)
{
    World world;

    // You likely already have this type
    RenderSystem renderSystem(renderer);

    ScenarioManager scenarios;

    scenarios.Add(std::make_unique<Scenario_PrimitiveScene>());

    // Pick a default scenario (first one)
    if (scenarios.Count() > 0)
        scenarios.SwitchTo(world, 0);

    bool simRunning = true;
    bool stepOnce = false;

    bool useFixedTimestep = true;
    float fixedDt = 1.0f / 60.0f;

    CameraRole activeCamera = CameraRole::Overview;

    auto moveActiveCamera = [&](float dt)
        {
            // Find active camera entity
            Entity camEntity = INVALID_ENTITY;
            world.forEach<CameraRoleComponent>([&](Entity e, CameraRoleComponent& r)
                {
                    if (r.role == activeCamera && camEntity == INVALID_ENTITY)
                        camEntity = e;
                });
            if (camEntity == INVALID_ENTITY) return;

            auto* tr = world.getComponent<TransformComponent>(camEntity);
            if (!tr) return;

            // --- Rotation with arrow keys ---
            const float lookSpeed = 1.5f; // radians/sec
            if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) tr->rotation.y -= lookSpeed * dt; // yaw -
            if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) tr->rotation.y += lookSpeed * dt; // yaw +
            if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) tr->rotation.x -= lookSpeed * dt; // pitch up (look up)
            if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) tr->rotation.x += lookSpeed * dt; // pitch down

            // Clamp pitch so it doesn't flip
            const float pitchLimit = glm::radians(89.0f);
            if (tr->rotation.x > pitchLimit) tr->rotation.x = pitchLimit;
            if (tr->rotation.x < -pitchLimit) tr->rotation.x = -pitchLimit;

            // --- Movement with WASD (relative to yaw) ---
            const float moveSpeed = 10.0f; // units/sec (tweak)
            glm::vec3 moveLocal(0.0f);

            if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) moveLocal.z += 1.0f;
            if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) moveLocal.z -= 1.0f;
            if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) moveLocal.x -= 1.0f;
            if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) moveLocal.x += 1.0f;

            // Optional vertical movement
            if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) moveLocal.y -= 1.0f;
            if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) moveLocal.y += 1.0f;

            if (moveLocal.x != 0.0f || moveLocal.y != 0.0f || moveLocal.z != 0.0f)
            {
                // Normalize so diagonal isn't faster
                moveLocal = glm::normalize(moveLocal);

                // Build forward/right from yaw only (keeps movement flat unless Q/E)
                float yaw = tr->rotation.y;
                glm::vec3 forward(std::sin(yaw), 0.0f, std::cos(yaw));
                glm::vec3 right(forward.z, 0.0f, -forward.x);
                glm::vec3 up(0.0f, 1.0f, 0.0f);

                glm::vec3 worldMove =
                    right * moveLocal.x +
                    up * moveLocal.y +
                    forward * moveLocal.z;

                tr->position += worldMove * (moveSpeed * dt);
            }
        };

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        float dtReal = GetDeltaTimeSeconds();
        float dt = useFixedTimestep ? fixedDt : dtReal;

        if (!ImGui::GetIO().WantCaptureKeyboard)
            moveActiveCamera(dtReal);

        // --- Scenario update ---
        if (Scenario* s = scenarios.Current())
        {
            if (simRunning)
            {
                s->Update(world, dt);
            }
            else if (stepOnce)
            {
                s->Update(world, dt);
                stepOnce = false;
            }
        }

        // ---- Begin UI frame (build main menu / scenario menu here) ----
        renderer.BeginImGuiFrame();

        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("Simulation"))
            {
                if (ImGui::MenuItem(simRunning ? "Pause" : "Start"))
                    simRunning = !simRunning;

                ImGui::Separator();
                ImGui::Checkbox("Use Fixed Timestep", &useFixedTimestep);
                ImGui::InputFloat("Fixed dt (seconds)", &fixedDt, 0.001f, 0.01f, "%.4f");
                if (fixedDt < 0.0001f) fixedDt = 0.0001f;

                ImGui::Text("~ %.1f Hz", 1.0f / fixedDt);

                ImGui::Separator();
                if (!simRunning && ImGui::MenuItem("Step One Timestep"))
                    stepOnce = true;

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("View"))
            {
                ImGui::ColorEdit3("Background", world.ClearColour());
                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("Camera"))
            {
                // Switch cameras
                bool isOverview = (activeCamera == CameraRole::Overview);
                bool isNavigation = (activeCamera == CameraRole::Navigation);

                if (ImGui::MenuItem("Overview", nullptr, isOverview))
                    activeCamera = CameraRole::Overview;

                if (ImGui::MenuItem("Navigation", nullptr, isNavigation))
                    activeCamera = CameraRole::Navigation;

                ImGui::Separator();

                // Move the active camera (edit the ECS transform)
                Entity camEntity = INVALID_ENTITY;
                world.forEach<CameraRoleComponent>([&](Entity e, CameraRoleComponent& r)
                    {
                        if (r.role == activeCamera && camEntity == INVALID_ENTITY)
                            camEntity = e;
                    });

                if (camEntity != INVALID_ENTITY)
                {
                    if (auto* tr = world.getComponent<TransformComponent>(camEntity))
                    {
                        ImGui::DragFloat3("Position", &tr->position.x, 0.1f);
                        ImGui::DragFloat3("Rotation (rad)", &tr->rotation.x, 0.02f);
                    }
                }
                else
                {
                    ImGui::TextUnformatted("No camera entity found for this role.");
                }

                ImGui::EndMenu();
            }

            if (Scenario* curr = scenarios.Current())
                curr->ImGuiMainMenu(world);

            ImGui::EndMainMenuBar();
        }

        renderer.EndImGuiFrame();
        // ---- End UI frame ----

        // --- Build + render scene ---
        renderSystem.render(world, activeCamera);
    }
    return 0;
}
