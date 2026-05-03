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

    // Cameras (keep as before)
    spawnCamera(CameraRole::Navigation,
        glm::vec3(0.0f, 3.0f, -8.0f),
        glm::vec3(-15.0f, 0.0f, 0.0f),
        60.0f);

    spawnCamera(CameraRole::Overview,
        glm::vec3(0.0f, 25.0f, 0.0f),
        glm::vec3(-90.0f, 0.0f, 0.0f),
        60.0f);

    // Directional light (keep)
    {
        Entity lightE = world.createEntity();
        DirectionalLightComponent sun{};
        sun.direction = glm::normalize(glm::vec3(-0.3f, -1.0f, -0.2f));
        sun.color = glm::vec3(1.0f);
        sun.intensity = 2.0f;
        world.addComponent(lightE, sun);
    }

    // --- Plane at y=0 ---
    {
        Entity e = world.createEntity();

        TransformComponent tr{};
        tr.position = glm::vec3(0.0f, 0.0f, 0.0f);
        tr.rotation = glm::vec3(0.0f);
        tr.scale = glm::vec3(10.0f);

        world.addComponent(e, tr);
        world.addComponent(e, RenderMeshComponent{ "Plane.obj", "RedChecker.png" });
        world.addComponent(e, MaterialComponent{});

        PlaneColliderComponent pc{};
        pc.normal = glm::vec3(0, 1, 0);
        world.addComponent(e, pc);

        // IMPORTANT: give the plane a static PhysicsComponent too (so later we can stop using fake static bodies)
        PhysicsComponent phys{};
        phys.body.SetMotionType(BodyMotionType::Static);
        phys.body.SetPosition(tr.position);
        phys.body.SetOrientation(glm::quat(1, 0, 0, 0));
        // material values shouldn't matter yet, but set sane defaults
        phys.restitution = 0.0f;
        phys.staticFriction = 0.8f;
        phys.dynamicFriction = 0.6f;

        world.addComponent(e, phys);
    }

    // --- Single sphere dropped gently (no initial velocity) ---
    {
        Entity e = world.createEntity();

        TransformComponent tr{};
        tr.scale = glm::vec3(1.0f);

        SphereColliderComponent sc{};
        sc.baseRadius = 0.5f;
        sc.localCenter = glm::vec3(0.0f);

        // Place it slightly above the plane so it starts NOT intersecting.
        tr.position = glm::vec3(0.0f, sc.baseRadius + 0.25f, 0.0f);
        tr.rotation = glm::vec3(0.0f);

        world.addComponent(e, tr);
        world.addComponent(e, RenderMeshComponent{ "sphere.obj", "sky3.png" });
        world.addComponent(e, MaterialComponent{});
        world.addComponent(e, sc);

        PhysicsComponent phys{};
        phys.body.SetMotionType(BodyMotionType::Dynamic);
        phys.body.SetMass(1.0f);
        phys.body.SetPosition(tr.position);
        phys.body.SetLinearVelocity(glm::vec3(0.0f));  // no initial velocity
        phys.body.SetAngularVelocity(glm::vec3(0.0f));

        // Make it non-bouncy to avoid micro-bounce hiding the real issue
        phys.restitution = 0.0f;
        phys.staticFriction = 0.8f;
        phys.dynamicFriction = 0.6f;

        world.addComponent(e, phys);
    }
}