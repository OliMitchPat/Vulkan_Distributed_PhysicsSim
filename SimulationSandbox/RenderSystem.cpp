#include "RenderSystem.h"
#include <glm/gtc/matrix_transform.hpp>

// Helper to find active camera entity by role
static Entity findActiveCameraEntity(World& world, CameraRole role)
{
    Entity result = INVALID_ENTITY;
    world.forEach<CameraRoleComponent>(
        [&](Entity e, CameraRoleComponent& roleComp)
        {
            if (roleComp.role == role && result == INVALID_ENTITY)
                result = e;
        }
    );
    return result;
}

CameraRenderData RenderSystem::buildCameraData(World& world, CameraRole activeCameraRole) const
{
    CameraRenderData camData{};

    Entity const camEntity = findActiveCameraEntity(world, activeCameraRole);
    if (camEntity == INVALID_ENTITY)
    {
        // Fallback: leave defaults
        return camData;
    }

    auto* const camComp = world.getComponent<CameraComponent>(camEntity);
    auto* const transform = world.getComponent<TransformComponent>(camEntity);
    if (!camComp || !transform)
    {
        return camData;
    }

    camData.position = transform->position;

    // Build view matrix from yaw/pitch Euler
    const float yaw = transform->rotation.y;
    const float pitch = transform->rotation.x;

    glm::vec3 const forward{
        std::cos(pitch) * std::sin(yaw),
        std::sin(pitch),
        std::cos(pitch) * std::cos(yaw)
    };

    glm::vec3 const target = transform->position + forward;
    glm::vec3 const up{ 0.0f, 1.0f, 0.0f };

    camData.view = glm::lookAt(transform->position, target, up);

    // Projection (Vulkan-style)
    float const aspect = renderer.getAspectRatio();
    float const fovRad = glm::radians(camComp->fov);

    camData.proj = glm::perspective(fovRad, aspect, camComp->nearClip, camComp->farClip);
    camData.proj[1][1] *= -1.0f; // GLM Y-flip for Vulkan

    return camData;
}

void RenderSystem::buildLights(World& world, RenderScene& scene) const
{
    world.forEach<DirectionalLightComponent>(
        [&](Entity, DirectionalLightComponent& light)
        {
            DirectionalLightRenderData data;
            data.direction = light.direction;
            data.color = light.color;
            data.intensity = light.intensity;
            scene.directionalLights.push_back(data);
        }
    );
}

void RenderSystem::buildSparkLights(World& world, RenderScene& scene) const
{
    scene.sparkLights.clear();
    scene.sparkLights.reserve(32);

    world.forEach<SparkLightComponent>(
        [&](Entity e, SparkLightComponent& s)
        {
            if (!s.active) return;

            auto* tr = world.getComponent<TransformComponent>(e);
            if (!tr) return;

            SparkLightRenderData out{};
            out.position = tr->position;
            out.radius = s.radius;
            out.color = s.color;
            out.intensity = s.intensity;

            scene.sparkLights.push_back(out);

            // Hard cap for GPU UBO
            if ((int)scene.sparkLights.size() >= 32)
                return;
        }
    );
}

void RenderSystem::buildInstances(World& world, RenderScene& scene) const
{
    // Entities must have: Transform + RenderMesh + Material
    world.forEach<RenderMeshComponent>(
        [&](Entity e, RenderMeshComponent& meshComp)
        {
            auto* transform = world.getComponent<TransformComponent>(e);
            auto* material = world.getComponent<MaterialComponent>(e);
            if (!transform || !material)
                return;

            RenderInstance inst{};

            // Model matrix from Transform
            glm::mat4 M{ 1.0f };
            M = glm::translate(M, transform->position);
            M = glm::rotate(M, transform->rotation.y, glm::vec3(0, 1, 0));
            M = glm::rotate(M, transform->rotation.x, glm::vec3(1, 0, 0));
            M = glm::rotate(M, transform->rotation.z, glm::vec3(0, 0, 1));
            M = glm::scale(M, transform->scale);
            inst.model = M;

            inst.meshName = meshComp.meshName;
            inst.textureName = meshComp.textureName;

            inst.shadingModel = material->shadingModel;
            inst.diffuseColor = material->diffuseColor;
            inst.specularColor = material->specularColor;
            inst.shininess = material->shininess;
            inst.castsShadows = material->castsShadows;
            inst.receivesShadows = material->receivesShadows;

            extern int gForceShading; // -1 none, 0 gouraud, 1 phong

            if (gForceShading == 0) inst.shadingModel = ShadingModel::Gouraud;
            if (gForceShading == 1) inst.shadingModel = ShadingModel::Phong;

            scene.instances.push_back(std::move(inst));
        }
    );
}

void RenderSystem::buildParticles(World& world, RenderScene& scene) const
{
    // Find environment singleton entity
    Entity envEntity = INVALID_ENTITY;
    EnvironmentStateComponent* env = nullptr;

    world.forEach<EnvironmentStateComponent>(
        [&](Entity e, EnvironmentStateComponent& envComp)
        {
            envEntity = e;
            env = &envComp;
        }
    );
    if (envEntity == INVALID_ENTITY) return;

    auto* const pool = world.getComponent<ParticlePoolComponent>(envEntity);
    if (!pool) return;

    scene.particles.clear();
    scene.particles.reserve(pool->particles.size());

    for (const SimParticle& p : pool->particles)
    {
        ParticleRenderData out{};
        out.position = p.position;

        float lifeT = (p.lifetime > 0.0f) ? (p.age / p.lifetime) : 1.0f;
        lifeT = glm::clamp(lifeT, 0.0f, 1.0f);

        // simple type-based styling (placeholder)
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

        scene.particles.push_back(out);
    }
}

void RenderSystem::render(World& world, CameraRole activeCameraRole)
{
    RenderScene scene{};

    scene.camera = buildCameraData(world, activeCameraRole);
    scene.ambientLight = glm::vec3(0.02f);
    buildLights(world, scene);
    buildInstances(world, scene);
    buildParticles(world, scene);
    buildSparkLights(world, scene);
    const float* c = world.ClearColour();
    scene.clearColor = glm::vec4(c[0], c[1], c[2], c[3]);
    renderer.render(scene);
}
