#include "Scenario_PrimitiveScene.h"
#include "World.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "Components.h"

void Scenario_PrimitiveScene::OnLoad(World& world)
{
    auto spawnCamera = [&](CameraRole role, glm::vec3 pos, glm::vec3 rotDeg, float fov)
        {
            Entity cam = world.createEntity();

            TransformComponent tr{};
            tr.position = pos;
            tr.rotation = glm::radians(rotDeg);

            CameraComponent cc{};
            cc.fov = fov;
            cc.nearClip = 0.1f;
            cc.farClip = 600.0f;

            CameraRoleComponent cr{};
            cr.role = role;

            world.addComponent(cam, tr);
            world.addComponent(cam, cc);
            world.addComponent(cam, cr);

            return cam;
        };

    // Camera
    spawnCamera(CameraRole::Navigation,
        glm::vec3(0.0f, 3.0f, -8.0f),
        glm::vec3(-15.0f, 0.0f, 0.0f),
        60.0f
    );

    spawnCamera(
        CameraRole::Overview,
        glm::vec3(0.0f, 25.0f, 0.0f),
        glm::vec3(-90.0f, 0.0f, 0.0f),
        60.0f
    );

    // Light


    // --- Directional light ---
    {
        Entity lightE = world.createEntity();

        DirectionalLightComponent sun{};
        sun.direction = glm::normalize(glm::vec3(-0.3f, -1.0f, -0.2f)); // points downwards-ish
        sun.color = glm::vec3(1.0f, 1.0f, 1.0f);
        sun.intensity = 2.0f; // try 1.0f–5.0f depending on your shader

        world.addComponent(lightE, sun);
    }

    // --- Plane ---
    {
        Entity e = world.createEntity();

        TransformComponent tr{};
        tr.position = glm::vec3(0.0f, 0.0f, 0.0f);
        tr.rotation = glm::vec3(0.0f);                 // radians
        tr.scale = glm::vec3(10.0f);

        world.addComponent(e, tr);

        // textureName can be "" if your shader/procedural material ignores textures
        world.addComponent(e, RenderMeshComponent{
            "Plane.obj",
            "RedChecker.png" 
            });

        world.addComponent(e, MaterialComponent{}); // defaults are fine

        world.addComponent(e, PlaneColliderComponent{ glm::normalize(glm::vec3(0,1,0)) });
    }

    // --- Sphere ---
    {
        Entity e = world.createEntity();

        TransformComponent tr{};
        tr.position = glm::vec3(5.0f, 2.0f, 0.0f);
        tr.rotation = glm::vec3(0.0f);
        tr.scale = glm::vec3(1.0f);

        world.addComponent(e, tr);

        world.addComponent(e, RenderMeshComponent{
            "sphere.obj",
            "sky3.png"
            });

        world.addComponent(e, MaterialComponent{});

        world.addComponent(e, SphereColliderComponent{ 0.5f });
        world.addComponent(e, PhysicsComponent(tr.position, glm::vec3(-2, 2, 0), 1.0f));
    }
    {
        Entity e = world.createEntity();

        TransformComponent tr{};
        tr.position = glm::vec3(0.0f, 1.0f, 0.0f);
        tr.rotation = glm::vec3(0.0f);
        tr.scale = glm::vec3(1.0f);

        world.addComponent(e, tr);

        world.addComponent(e, RenderMeshComponent{
            "sphere.obj",
            "sky3.png"
            });

        world.addComponent(e, MaterialComponent{});

        world.addComponent(e, SphereColliderComponent{ 0.5f });
        world.addComponent(e, PhysicsComponent(tr.position, glm::vec3(0, 0, 0), 1.0f));
    }
}