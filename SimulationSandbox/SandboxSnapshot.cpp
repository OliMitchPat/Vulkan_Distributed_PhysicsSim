#include "SandboxSnapshot.h"

#include "Components.h"
#include "World.h"

#include <glm/gtc/matrix_transform.hpp>

#include <type_traits>
#include <variant>

namespace
{
glm::vec3 OwnerColor(int ownerId)
{
    if (ownerId < 0)
        return glm::vec3(0.85f);

    static const glm::vec3 k[] = {
        {1.0f, 0.2f, 0.2f},
        {0.2f, 1.0f, 0.2f},
        {0.2f, 0.4f, 1.0f},
        {1.0f, 1.0f, 0.2f},
    };
    return k[ownerId % static_cast<int>(sizeof(k) / sizeof(k[0]))];
}
}

const char* DisplayModeName(int mode)
{
    return mode == 1 ? "Owner Colours" : "Material Colours";
}

std::shared_ptr<WorldSnapshot> CaptureSnapshot(
    World& world,
    uint64_t tickNumber,
    int displayMode,
    const glm::vec4& clearColor)
{
    auto snap = std::make_shared<WorldSnapshot>();
    snap->simTickNumber = tickNumber;
    snap->clearColor = clearColor;
    snap->ambientLight = glm::vec3(0.02f);

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
                            visualScale *= glm::vec3(s.radius);
                        }
                        else if constexpr (std::is_same_v<T, CuboidShape>)
                        {
                            visualScale *= (0.5f * s.size);
                        }
                        else if constexpr (std::is_same_v<T, CylinderShape>)
                        {
                            visualScale *= glm::vec3(s.radius, 0.5f * s.height, s.radius);
                        }
                        else if constexpr (std::is_same_v<T, CapsuleShape>)
                        {
                            constexpr float kMeshRadius = 2.0f;
                            constexpr float kMeshHeight = 5.0f;
                            const float desiredRadius = s.radius;
                            const float desiredTotalHeight = s.height + 2.0f * s.radius;

                            visualScale.x *= desiredRadius / kMeshRadius;
                            visualScale.z *= desiredRadius / kMeshRadius;
                            visualScale.y *= desiredTotalHeight / kMeshHeight;
                        }
                        else if constexpr (std::is_same_v<T, PlaneShape>)
                        {
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
            inst.diffuseColor = glm::vec4(mat->diffuseColor, mat->alpha);
            inst.specularColor = mat->specularColor;
            inst.shininess = mat->shininess;
            inst.castsShadows = mat->castsShadows;
            inst.receivesShadows = mat->receivesShadows;

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

            snap->instances.push_back(std::move(inst));
        });

    world.forEach<DirectionalLightComponent>([&](Entity, DirectionalLightComponent& light)
        {
            DirectionalLightRenderData d;
            d.direction = light.direction;
            d.color = light.color;
            d.intensity = light.intensity;
            snap->directionalLights.push_back(d);
        });

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
            if (static_cast<int>(snap->sparkLights.size()) >= 32) return;
        });

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
