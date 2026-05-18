#pragma once

#include "Components.h"
#include "RenderScene.h"
#include "World.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <string>
#include <vector>

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

struct SceneCameraOption
{
    Entity entity = INVALID_ENTITY;
    std::string name;
    CameraComponent::Projection projection = CameraComponent::Projection::Perspective;
};

struct SceneBounds
{
    glm::vec3 min{ 0.0f };
    glm::vec3 max{ 0.0f };
    bool valid = false;
};

CameraRenderData BuildCameraRenderData(const RenderCamera& cam, float aspect);
std::vector<SceneCameraOption> CollectSceneCameras(World& world);
glm::vec3 EstimateSceneCenter(World& world);
SceneBounds EstimateSceneBounds(World& world);
void FitOrthographicCameraToScene(World& world, RenderCamera& renderCamera);
bool ApplySceneCamera(World& world, Entity cameraEntity, RenderCamera& renderCamera);
const char* CameraProjectionName(CameraComponent::Projection projection);
