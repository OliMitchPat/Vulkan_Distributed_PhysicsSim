#pragma once

#include "EcsTypes.h"

#include <glm/glm.hpp>

enum class FlockingObstacleType
{
    Sphere,
    Aabb,
    Plane
};

enum class FlockingNeighbourSearchMode
{
    BruteForce = 0,
    UniformGrid = 1,
    Octree = 2,
    GpuComputeBruteForce = 3
};

struct FlockingStats
{
    int agentCount = 0;
    int ownedAgentCount = 0;
    int neighbourChecks = 0;
    int neighboursFound = 0;
    int collisionAvoidanceChecks = 0;
    int spatialCellsOrNodes = 0;
    int spatialCandidateChecks = 0;
    size_t memoryEstimateBytes = 0;
    FlockingNeighbourSearchMode searchMode = FlockingNeighbourSearchMode::BruteForce;
    bool gpuComputeAvailable = false;
    bool gpuFallbackActive = false;
    float updateMs = 0.0f;
    float spatialBuildMs = 0.0f;
    float neighbourSearchMs = 0.0f;
    float gpuUploadMs = 0.0f;
    float gpuDispatchMs = 0.0f;
    float gpuReadbackMs = 0.0f;
    float gpuTotalMs = 0.0f;
    float cohesionMs = 0.0f;
    float alignmentMs = 0.0f;
    float separationMs = 0.0f;
    float avoidanceMs = 0.0f;
};

struct FlockingDebugAgent
{
    Entity entity = INVALID_ENTITY;
    glm::vec3 position{ 0.0f };
    glm::vec3 velocity{ 0.0f };
    glm::vec3 cohesionForce{ 0.0f };
    glm::vec3 alignmentForce{ 0.0f };
    glm::vec3 separationForce{ 0.0f };
    glm::vec3 avoidanceForce{ 0.0f };
    glm::vec3 totalForce{ 0.0f };
    int neighbourCount = 0;
};
