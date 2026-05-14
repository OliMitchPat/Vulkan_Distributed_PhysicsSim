#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <string>
#include "Components.h"    // for ShadingModel

// Camera data for one frame
struct CameraRenderData
{
    glm::mat4 view{ 1.0f };
    glm::mat4 proj{ 1.0f };
    glm::vec3 position{ 0.0f };
};

// Directional light data for shaders
struct DirectionalLightRenderData
{
    glm::vec3 direction{ 0.0f, -1.0f, 0.0f };
    glm::vec3 color{ 1.0f, 1.0f, 1.0f };
    float intensity = 1.0f;
};

struct Particle
{
    glm::vec3 position;
    glm::vec3 velocity;
    float age = 0.0f;
    float lifetime = 1.0f;
};

struct ParticleRenderData
{
    glm::vec3 position;
    float size;
    glm::vec4 color;
};

struct SparkLightRenderData
{
    glm::vec3 position{ 0.0f };
    float radius = 3.0f;

    glm::vec3 color{ 1.0f, 0.8f, 0.4f };
    float intensity = 5.0f;
};


// One drawable instance in the world
struct RenderInstance
{
    glm::mat4 model{ 1.0f };

    std::string meshName;       // from RenderMeshComponent
    std::string textureName;    // from RenderMeshComponent

    ShadingModel shadingModel = ShadingModel::Phong;
    glm::vec4 diffuseColor{ 1.0f, 1.0f, 1.0f, 1.0f };   
    glm::vec3 specularColor{ 1.0f, 1.0f, 1.0f };
    float shininess = 32.0f;

    bool castsShadows = true;
    bool receivesShadows = true;
};

// Everything the renderer needs per frame
struct RenderScene
{
    CameraRenderData camera;
    glm::vec3 ambientLight{ 0.1f, 0.1f, 0.1f };
    std::vector<DirectionalLightRenderData> directionalLights;
    std::vector<RenderInstance> instances;
    std::vector<ParticleRenderData> particles;
    std::vector<SparkLightRenderData> sparkLights;
    glm::vec4 clearColor{ 0.1f, 0.1f, 0.1f, 1.0f };
    // Later: particle render data, shadow maps, etc.
};
