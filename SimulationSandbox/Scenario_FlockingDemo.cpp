#include "Scenario_FlockingDemo.h"

#include "Components.h"
#include "World.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <random>
#include <string>
#include <algorithm>

namespace
{
    glm::quat LookAtOrientation(const glm::vec3& from, const glm::vec3& to)
    {
        const glm::vec3 forward = glm::normalize(to - from);
        const glm::mat4 invView = glm::inverse(glm::lookAt(from, to, glm::vec3(0, 1, 0)));
        return glm::normalize(glm::quat_cast(invView));
    }

    float RandomRange(std::mt19937& rng, float minValue, float maxValue)
    {
        std::uniform_real_distribution<float> dist(minValue, maxValue);
        return dist(rng);
    }

    glm::vec3 RandomVec3(std::mt19937& rng, const glm::vec3& minValue, const glm::vec3& maxValue)
    {
        return glm::vec3(
            RandomRange(rng, minValue.x, maxValue.x),
            RandomRange(rng, minValue.y, maxValue.y),
            RandomRange(rng, minValue.z, maxValue.z));
    }

    MaterialComponent MakeMaterial(const glm::vec3& colour, float shininess = 24.0f, float alpha = 1.0f)
    {
        MaterialComponent mat{};
        mat.diffuseColor = colour;
        mat.specularColor = glm::vec3(0.8f);
        mat.shininess = shininess;
        mat.alpha = alpha;
        return mat;
    }

    void AddSceneCamera(
        World& world,
        const std::string& name,
        const glm::vec3& position,
        const glm::vec3& target,
        float fov,
        float farClip)
    {
        Entity camera = world.createEntity();

        TransformComponent tr{};
        tr.position = position;
        tr.orientation = LookAtOrientation(position, target);
        tr.rotation = glm::eulerAngles(tr.orientation);

        CameraComponent cam{};
        cam.projection = CameraComponent::Projection::Perspective;
        cam.fov = fov;
        cam.nearClip = 0.1f;
        cam.farClip = farClip;

        world.addComponent(camera, NameComponent{ name });
        world.addComponent(camera, tr);
        world.addComponent(camera, cam);
    }

    void AddSphereObstacle(World& world, const glm::vec3& position, float radius, const glm::vec3& colour)
    {
        Entity e = world.createEntity();

        TransformComponent tr{};
        tr.position = position;
        tr.scale = glm::vec3(radius);

        ShapeComponent shape{};
        shape.shape = SphereShape{ radius };

        world.addComponent(e, NameComponent{ "flocking sphere obstacle" });
        world.addComponent(e, tr);
        world.addComponent(e, RenderMeshComponent{ "Sphere.obj", "white.png" });
        world.addComponent(e, MakeMaterial(colour));
        world.addComponent(e, shape);
    }

    void AddCuboidObstacle(World& world, const glm::vec3& position, const glm::vec3& size, const glm::vec3& colour)
    {
        Entity e = world.createEntity();

        TransformComponent tr{};
        tr.position = position;
        tr.scale = size;

        ShapeComponent shape{};
        shape.shape = CuboidShape{ size };

        world.addComponent(e, NameComponent{ "flocking cuboid obstacle" });
        world.addComponent(e, tr);
        world.addComponent(e, RenderMeshComponent{ "cube.obj", "white.png" });
        world.addComponent(e, MakeMaterial(colour));
        world.addComponent(e, shape);
    }
}

void Scenario_FlockingDemo::OnLoad(World& world)
{
    CreateManualFlockingScene(world, m_requestedBoidCount.load());
}

void Scenario_FlockingDemo::Update(World& world, float, uint32_t)
{
    const int requested = std::clamp(m_requestedBoidCount.load(), 10, 5000);
    if (requested != m_currentBoidCount)
        CreateManualFlockingScene(world, requested);
}

void Scenario_FlockingDemo::RequestBoidCount(int count)
{
    m_requestedBoidCount.store(std::clamp(count, 10, 5000));
}

void Scenario_FlockingDemo::CreateManualFlockingScene(World& world, int boidCount)
{
    world.Clear();
    m_currentBoidCount = std::clamp(boidCount, 10, 5000);

    world.ClearColour()[0] = 0.035f;
    world.ClearColour()[1] = 0.045f;
    world.ClearColour()[2] = 0.055f;
    world.ClearColour()[3] = 1.0f;

    AddSceneCamera(
        world,
        "Flocking Arena Camera",
        glm::vec3(0.0f, 18.0f, 35.0f),
        glm::vec3(0.0f, 8.0f, 0.0f),
        55.0f,
        160.0f);

    AddSceneCamera(
        world,
        "Flocking Top Camera",
        glm::vec3(0.0f, 35.0f, 0.01f),
        glm::vec3(0.0f, 8.0f, 0.0f),
        55.0f,
        160.0f);

    {
        Entity lightE = world.createEntity();
        DirectionalLightComponent sun{};
        sun.direction = glm::normalize(glm::vec3(-0.35f, -1.0f, -0.25f));
        sun.color = glm::vec3(1.0f);
        sun.intensity = 2.4f;
        world.addComponent(lightE, sun);
    }

    {
        Entity e = world.createEntity();
        TransformComponent tr{};
        tr.position = glm::vec3(0.0f);
        tr.scale = glm::vec3(40.0f, 1.0f, 40.0f);

        ShapeComponent shape{};
        shape.shape = PlaneShape{ glm::vec3(0, 1, 0) };

        world.addComponent(e, NameComponent{ "flocking floor" });
        world.addComponent(e, tr);
        world.addComponent(e, RenderMeshComponent{ "Plane.obj", "checkers.png" });
        world.addComponent(e, MakeMaterial(glm::vec3(0.25f, 0.28f, 0.30f), 12.0f));
        world.addComponent(e, shape);
    }

    AddSphereObstacle(world, glm::vec3(-5.5f, 6.0f, -2.0f), 2.4f, glm::vec3(1.0f, 0.45f, 0.16f));
    AddSphereObstacle(world, glm::vec3(6.0f, 8.0f, 4.0f), 2.0f, glm::vec3(0.9f, 0.25f, 0.35f));
    AddCuboidObstacle(world, glm::vec3(1.5f, 5.5f, -7.0f), glm::vec3(2.5f, 5.0f, 2.5f), glm::vec3(0.5f, 0.75f, 1.0f));

    std::mt19937 rng{ 1337u };
    const glm::vec3 spawnMin{ -10.0f, 2.0f, -10.0f };
    const glm::vec3 spawnMax{ 10.0f, 12.0f, 10.0f };

    for (int i = 0; i < m_currentBoidCount; ++i)
    {
        Entity e = world.createEntity();

        TransformComponent tr{};
        tr.position = RandomVec3(rng, spawnMin, spawnMax);
        tr.scale = glm::vec3(0.22f);

        glm::vec3 velocityDir = RandomVec3(rng, glm::vec3(-1.0f), glm::vec3(1.0f));
        if (glm::dot(velocityDir, velocityDir) < 1e-5f)
            velocityDir = glm::vec3(1, 0, 0);
        velocityDir = glm::normalize(velocityDir);

        VelocityComponent velocity{};
        velocity.linearVelocity = velocityDir * RandomRange(rng, 3.0f, 7.0f);

        FlockingComponent flock{};
        flock.maxSpeed = 8.0f;
        flock.maxForce = 15.0f;
        flock.perceptionRadius = 6.0f;
        flock.separationRadius = 1.5f;
        flock.cohesionWeight = 0.8f;
        flock.alignmentWeight = 1.0f;
        flock.separationWeight = 1.5f;
        flock.avoidanceWeight = 2.5f;
        flock.boidRadius = 0.22f;

        ShapeComponent shape{};
        shape.shape = SphereShape{ 0.22f };

        const float hue = static_cast<float>(i % 4);
        const glm::vec3 colour =
            hue < 1.0f ? glm::vec3(0.2f, 0.85f, 1.0f) :
            hue < 2.0f ? glm::vec3(0.55f, 1.0f, 0.35f) :
            hue < 3.0f ? glm::vec3(1.0f, 0.9f, 0.25f) :
                         glm::vec3(1.0f, 0.45f, 0.75f);

        world.addComponent(e, NameComponent{ "boid " + std::to_string(i + 1) });
        world.addComponent(e, tr);
        world.addComponent(e, velocity);
        world.addComponent(e, flock);
        world.addComponent(e, ShapeComponent{ shape });
        world.addComponent(e, RenderMeshComponent{ "Sphere.obj", "white.png" });
        world.addComponent(e, MakeMaterial(colour, 32.0f));
        world.addComponent(e, OwnerComponent{ -1 });
    }
}
