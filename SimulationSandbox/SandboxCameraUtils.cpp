#include "SandboxCameraUtils.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

CameraRenderData BuildCameraRenderData(const RenderCamera& cam, float aspect)
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

std::vector<SceneCameraOption> CollectSceneCameras(World& world)
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

glm::vec3 EstimateSceneCenter(World& world)
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

SceneBounds EstimateSceneBounds(World& world)
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

void FitOrthographicCameraToScene(World& world, RenderCamera& renderCamera)
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

bool ApplySceneCamera(World& world, Entity cameraEntity, RenderCamera& renderCamera)
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

const char* CameraProjectionName(CameraComponent::Projection projection)
{
    return projection == CameraComponent::Projection::Orthographic
        ? "Orthographic"
        : "Perspective";
}
