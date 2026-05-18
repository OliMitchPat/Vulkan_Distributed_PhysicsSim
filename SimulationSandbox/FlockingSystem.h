#pragma once

#include "FlockingTypes.h"

#include <glm/glm.hpp>
#include <array>
#include <unordered_map>
#include <vector>

class World;
class GpuFlockingCompute;
struct FlockingComponent;

class FlockingSystem
{
public:
    void Update(World& world, float dt);

    void SetEnabled(bool enabled) { m_enabled = enabled; }
    bool IsEnabled() const { return m_enabled; }

    void SetLocalPeerId(int peerId_1_to_4) { m_localPeerId = peerId_1_to_4; }
    int GetLocalPeerId() const { return m_localPeerId; }

    const FlockingStats& GetStats() const { return m_stats; }
    const std::vector<FlockingDebugAgent>& GetDebugAgents() const { return m_debugAgents; }

    void SetDebugEnabled(bool enabled) { m_debugEnabled = enabled; }
    bool IsDebugEnabled() const { return m_debugEnabled; }

    void SetBoundsEnabled(bool enabled) { m_boundsEnabled = enabled; }
    bool IsBoundsEnabled() const { return m_boundsEnabled; }

    void SetBounds(glm::vec3 minBounds, glm::vec3 maxBounds);

    void SetNeighbourSearchMode(FlockingNeighbourSearchMode mode) { m_searchMode = mode; }
    FlockingNeighbourSearchMode GetNeighbourSearchMode() const { return m_searchMode; }

    void SetGpuCompute(GpuFlockingCompute* compute) { m_gpuCompute = compute; }

private:
    struct BoidRef
    {
        Entity entity = INVALID_ENTITY;
        glm::vec3 position{ 0.0f };
        glm::vec3 velocity{ 0.0f };
        float boidRadius = 0.2f;
        bool updateable = true;
    };

    struct ObstacleProxy
    {
        Entity entity = INVALID_ENTITY;
        FlockingObstacleType type = FlockingObstacleType::Sphere;
        glm::vec3 position{ 0.0f };
        float radius = 1.0f;
        glm::vec3 min{ 0.0f };
        glm::vec3 max{ 0.0f };
        glm::vec3 normal{ 0.0f, 1.0f, 0.0f };
        bool enabled = true;
    };

    struct GridKey
    {
        int x = 0;
        int y = 0;
        int z = 0;

        bool operator==(const GridKey& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct GridKeyHash
    {
        size_t operator()(const GridKey& key) const;
    };

    struct OctreeNode
    {
        glm::vec3 min{ 0.0f };
        glm::vec3 max{ 0.0f };
        std::vector<size_t> indices;
        std::array<int, 8> children{};
        bool leaf = true;
    };

    void BuildBoidLists(World& world);
    void BuildObstacleList(World& world);
    void BuildSpatialIndex();
    void BuildUniformGrid();
    void BuildOctree();
    void InsertOctreeIndex(int nodeIndex, size_t boidIndex, int depth);
    void QueryNeighbours(World& world, const BoidRef& boid, const FlockingComponent& flock, std::vector<const BoidRef*>& perceptionNeighbours, std::vector<const BoidRef*>& separationNeighbours);
    void QueryNeighboursBruteForce(World& world, const BoidRef& boid, const FlockingComponent& flock, std::vector<const BoidRef*>& perceptionNeighbours, std::vector<const BoidRef*>& separationNeighbours);
    void QueryNeighboursUniformGrid(World& world, const BoidRef& boid, const FlockingComponent& flock, std::vector<const BoidRef*>& perceptionNeighbours, std::vector<const BoidRef*>& separationNeighbours);
    void QueryNeighboursOctree(World& world, const BoidRef& boid, const FlockingComponent& flock, std::vector<const BoidRef*>& perceptionNeighbours, std::vector<const BoidRef*>& separationNeighbours);
    void QueryOctreeNode(int nodeIndex, const glm::vec3& center, float radiusSq, std::vector<size_t>& candidates) const;
    void TestNeighbourCandidate(World& world, const BoidRef& boid, const FlockingComponent& flock, size_t candidateIndex, std::vector<const BoidRef*>& perceptionNeighbours, std::vector<const BoidRef*>& separationNeighbours);
    bool UpdateGpuCompute(World& world, float dt);

    glm::vec3 CalculateCohesion(const BoidRef& boid, const std::vector<const BoidRef*>& neighbours, float maxSpeed, float maxForce) const;
    glm::vec3 CalculateAlignment(const BoidRef& boid, const std::vector<const BoidRef*>& neighbours, float maxSpeed, float maxForce) const;
    glm::vec3 CalculateSeparation(const BoidRef& boid, const std::vector<const BoidRef*>& neighbours, float maxSpeed, float maxForce) const;
    glm::vec3 CalculateAvoidance(const BoidRef& boid, float maxSpeed, float maxForce);

    static glm::vec3 ClampLength(const glm::vec3& v, float maxLength);
    static glm::vec3 SafeNormalize(const glm::vec3& v, const glm::vec3& fallback = glm::vec3(0.0f));
    static glm::vec3 ClosestPointOnSegment(const glm::vec3& a, const glm::vec3& b, const glm::vec3& point);
    static void WeightedTruncatedAdd(glm::vec3& total, float& remaining, const glm::vec3& force);

    bool ShouldUpdateEntity(World& world, Entity e) const;

private:
    bool m_enabled = true;
    bool m_debugEnabled = false;
    bool m_boundsEnabled = true;
    FlockingNeighbourSearchMode m_searchMode = FlockingNeighbourSearchMode::BruteForce;
    GpuFlockingCompute* m_gpuCompute = nullptr;
    int m_localPeerId = 0;

    glm::vec3 m_boundsMin{ -20.0f, 0.0f, -20.0f };
    glm::vec3 m_boundsMax{ 20.0f, 20.0f, 20.0f };
    float m_boundsMargin = 3.0f;
    float m_boundsAvoidanceStrength = 20.0f;
    float m_obstacleMargin = 1.0f;
    float m_lookAheadDistance = 4.0f;

    std::vector<BoidRef> m_allBoids;
    std::vector<size_t> m_updateBoidIndices;
    std::vector<ObstacleProxy> m_obstacles;
    std::unordered_map<GridKey, std::vector<size_t>, GridKeyHash> m_uniformGrid;
    std::vector<OctreeNode> m_octreeNodes;
    std::vector<FlockingDebugAgent> m_debugAgents;
    FlockingStats m_stats{};
};
