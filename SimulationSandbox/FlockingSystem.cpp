#include "FlockingSystem.h"

#include "Components.h"
#include "World.h"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <type_traits>
#include <unordered_set>
#include <variant>

namespace
{
    constexpr float kEpsilon = 1e-5f;
    constexpr int kOctreeMaxDepth = 6;
    constexpr int kOctreeLeafCapacity = 16;

    float MaxAbsScale(const glm::vec3& scale)
    {
        return std::max(std::abs(scale.x), std::max(std::abs(scale.y), std::abs(scale.z)));
    }

    bool SphereIntersectsAabb(const glm::vec3& center, float radiusSq, const glm::vec3& minBounds, const glm::vec3& maxBounds)
    {
        const glm::vec3 closest = glm::clamp(center, minBounds, maxBounds);
        const glm::vec3 delta = center - closest;
        return glm::dot(delta, delta) <= radiusSq;
    }

    float RadiusForMode(const FlockingComponent& flock)
    {
        return std::max(flock.perceptionRadius, flock.separationRadius);
    }

    size_t ModeScratchMemory(size_t boidCount)
    {
        return boidCount * sizeof(size_t);
    }
}

size_t FlockingSystem::GridKeyHash::operator()(const GridKey& key) const
{
    const uint32_t x = static_cast<uint32_t>(key.x) * 73856093u;
    const uint32_t y = static_cast<uint32_t>(key.y) * 19349663u;
    const uint32_t z = static_cast<uint32_t>(key.z) * 83492791u;
    return static_cast<size_t>(x ^ y ^ z);
}

void FlockingSystem::SetBounds(glm::vec3 minBounds, glm::vec3 maxBounds)
{
    m_boundsMin = minBounds;
    m_boundsMax = maxBounds;
}

void FlockingSystem::Update(World& world, float dt)
{
    const auto start = std::chrono::steady_clock::now();
    m_stats = FlockingStats{};
    m_debugAgents.clear();

    if (!m_enabled || dt <= 0.0f)
        return;

    BuildBoidLists(world);
    BuildObstacleList(world);

    m_stats.agentCount = static_cast<int>(m_allBoids.size());
    m_stats.ownedAgentCount = static_cast<int>(m_updateBoidIndices.size());
    m_stats.searchMode = m_searchMode;

    const auto spatialStart = std::chrono::steady_clock::now();
    BuildSpatialIndex();
    const auto spatialEnd = std::chrono::steady_clock::now();
    m_stats.spatialBuildMs = std::chrono::duration<float, std::milli>(spatialEnd - spatialStart).count();

    std::vector<BoidRef> nextBoids = m_allBoids;

    for (const size_t boidIndex : m_updateBoidIndices)
    {
        BoidRef& boid = nextBoids[boidIndex];
        auto* transform = world.getComponent<TransformComponent>(boid.entity);
        auto* velocity = world.getComponent<VelocityComponent>(boid.entity);
        auto* flock = world.getComponent<FlockingComponent>(boid.entity);
        if (!transform || !velocity || !flock || !flock->enabled)
            continue;

        std::vector<const BoidRef*> perceptionNeighbours;
        std::vector<const BoidRef*> separationNeighbours;
        const auto neighbourStart = std::chrono::steady_clock::now();
        QueryNeighbours(world, boid, *flock, perceptionNeighbours, separationNeighbours);
        const auto neighbourEnd = std::chrono::steady_clock::now();
        m_stats.neighbourSearchMs += std::chrono::duration<float, std::milli>(neighbourEnd - neighbourStart).count();

        const glm::vec3 avoidance = CalculateAvoidance(boid, flock->maxSpeed, flock->maxForce);
        const glm::vec3 separation = CalculateSeparation(boid, separationNeighbours, flock->maxSpeed, flock->maxForce);
        const glm::vec3 alignment = CalculateAlignment(boid, perceptionNeighbours, flock->maxSpeed, flock->maxForce);
        const glm::vec3 cohesion = CalculateCohesion(boid, perceptionNeighbours, flock->maxSpeed, flock->maxForce);

        glm::vec3 totalForce{ 0.0f };
        float remaining = std::max(0.0f, flock->maxForce);

        WeightedTruncatedAdd(totalForce, remaining, avoidance * flock->avoidanceWeight);
        WeightedTruncatedAdd(totalForce, remaining, separation * flock->separationWeight);
        WeightedTruncatedAdd(totalForce, remaining, alignment * flock->alignmentWeight);
        WeightedTruncatedAdd(totalForce, remaining, cohesion * flock->cohesionWeight);

        glm::vec3 newVelocity = velocity->linearVelocity + totalForce * dt;
        newVelocity = ClampLength(newVelocity, flock->maxSpeed);

        glm::vec3 newPosition = transform->position + newVelocity * dt;
        if (m_boundsEnabled)
        {
            newPosition = glm::clamp(newPosition, m_boundsMin, m_boundsMax);
        }

        velocity->linearVelocity = newVelocity;
        transform->position = newPosition;
        if (glm::dot(newVelocity, newVelocity) > kEpsilon)
        {
            const float yaw = std::atan2(newVelocity.x, newVelocity.z);
            transform->rotation.y = yaw;
            transform->orientation = glm::angleAxis(yaw, glm::vec3(0, 1, 0));
        }

        boid.position = newPosition;
        boid.velocity = newVelocity;

        if (m_debugEnabled || flock->debugEnabled)
        {
            FlockingDebugAgent debug{};
            debug.entity = boid.entity;
            debug.position = newPosition;
            debug.velocity = newVelocity;
            debug.cohesionForce = cohesion;
            debug.alignmentForce = alignment;
            debug.separationForce = separation;
            debug.avoidanceForce = avoidance;
            debug.totalForce = totalForce;
            debug.neighbourCount = static_cast<int>(perceptionNeighbours.size());
            m_debugAgents.push_back(debug);
        }
    }

    m_allBoids = std::move(nextBoids);

    const auto end = std::chrono::steady_clock::now();
    m_stats.updateMs = std::chrono::duration<float, std::milli>(end - start).count();
}

void FlockingSystem::BuildBoidLists(World& world)
{
    m_allBoids.clear();
    m_updateBoidIndices.clear();

    world.forEach<FlockingComponent>([&](Entity e, FlockingComponent& flock)
        {
            if (!flock.enabled)
                return;

            auto* transform = world.getComponent<TransformComponent>(e);
            auto* velocity = world.getComponent<VelocityComponent>(e);
            if (!transform || !velocity)
                return;

            BoidRef boid{};
            boid.entity = e;
            boid.position = transform->position;
            boid.velocity = velocity->linearVelocity;
            boid.boidRadius = flock.boidRadius;
            boid.updateable = ShouldUpdateEntity(world, e);

            const size_t index = m_allBoids.size();
            m_allBoids.push_back(boid);
            if (boid.updateable)
                m_updateBoidIndices.push_back(index);
        });
}

void FlockingSystem::BuildObstacleList(World& world)
{
    m_obstacles.clear();

    world.forEach<ShapeComponent>([&](Entity e, ShapeComponent& shape)
        {
            if (world.getComponent<FlockingComponent>(e))
                return;

            auto* transform = world.getComponent<TransformComponent>(e);
            if (!transform)
                return;

            const auto* phys = world.getComponent<PhysicsComponent>(e);
            const bool isStatic = phys && phys->behaviour == BodyMotionType::Static;
            const bool isContainer = shape.collisionType == CollisionType::CONTAINER;
            const bool likelyStatic = !world.getComponent<VelocityComponent>(e);
            if (!isStatic && !isContainer && !likelyStatic)
                return;

            ObstacleProxy obstacle{};
            obstacle.entity = e;
            obstacle.position = transform->position;

            std::visit([&](const auto& s)
                {
                    using T = std::decay_t<decltype(s)>;
                    if constexpr (std::is_same_v<T, SphereShape>)
                    {
                        obstacle.type = FlockingObstacleType::Sphere;
                        obstacle.radius = s.radius * std::max(0.01f, MaxAbsScale(transform->scale));
                    }
                    else if constexpr (std::is_same_v<T, CuboidShape>)
                    {
                        obstacle.type = FlockingObstacleType::Aabb;
                        const glm::vec3 half = glm::abs(s.size * transform->scale) * 0.5f;
                        obstacle.min = transform->position - half;
                        obstacle.max = transform->position + half;
                    }
                    else if constexpr (std::is_same_v<T, PlaneShape>)
                    {
                        obstacle.type = FlockingObstacleType::Plane;
                        obstacle.normal = SafeNormalize(
                            glm::mat3_cast(transform->orientation) * s.normal,
                            glm::vec3(0, 1, 0));
                    }
                    else
                    {
                        obstacle.type = FlockingObstacleType::Sphere;
                        obstacle.radius = std::max(0.5f, MaxAbsScale(transform->scale));
                    }
                }, shape.shape);

            m_obstacles.push_back(obstacle);
        });
}

void FlockingSystem::BuildSpatialIndex()
{
    m_uniformGrid.clear();
    m_octreeNodes.clear();

    if (m_searchMode == FlockingNeighbourSearchMode::UniformGrid)
    {
        BuildUniformGrid();
    }
    else if (m_searchMode == FlockingNeighbourSearchMode::Octree)
    {
        BuildOctree();
    }
    else
    {
        m_stats.memoryEstimateBytes =
            m_allBoids.capacity() * sizeof(BoidRef) +
            m_updateBoidIndices.capacity() * sizeof(size_t) +
            ModeScratchMemory(m_allBoids.size());
    }
}

void FlockingSystem::BuildUniformGrid()
{
    const float cellSize = std::max(0.5f, m_stats.agentCount > 0 ? 6.0f : 1.0f);
    for (size_t i = 0; i < m_allBoids.size(); ++i)
    {
        const glm::vec3 p = m_allBoids[i].position / cellSize;
        GridKey key{};
        key.x = static_cast<int>(std::floor(p.x));
        key.y = static_cast<int>(std::floor(p.y));
        key.z = static_cast<int>(std::floor(p.z));
        m_uniformGrid[key].push_back(i);
    }

    m_stats.spatialCellsOrNodes = static_cast<int>(m_uniformGrid.size());
    size_t bucketMemory = 0;
    for (const auto& bucket : m_uniformGrid)
        bucketMemory += sizeof(bucket.first) + sizeof(bucket.second) + bucket.second.capacity() * sizeof(size_t);

    m_stats.memoryEstimateBytes =
        m_allBoids.capacity() * sizeof(BoidRef) +
        m_updateBoidIndices.capacity() * sizeof(size_t) +
        bucketMemory;
}

void FlockingSystem::BuildOctree()
{
    if (m_allBoids.empty())
    {
        m_stats.memoryEstimateBytes = 0;
        return;
    }

    glm::vec3 minBounds = m_allBoids.front().position;
    glm::vec3 maxBounds = m_allBoids.front().position;
    for (const auto& boid : m_allBoids)
    {
        minBounds = glm::min(minBounds, boid.position);
        maxBounds = glm::max(maxBounds, boid.position);
    }

    glm::vec3 center = (minBounds + maxBounds) * 0.5f;
    glm::vec3 half = (maxBounds - minBounds) * 0.5f;
    const float maxHalf = std::max(1.0f, std::max(half.x, std::max(half.y, half.z)) + 0.5f);
    minBounds = center - glm::vec3(maxHalf);
    maxBounds = center + glm::vec3(maxHalf);

    OctreeNode root{};
    root.min = minBounds;
    root.max = maxBounds;
    root.children.fill(-1);
    m_octreeNodes.push_back(std::move(root));

    for (size_t i = 0; i < m_allBoids.size(); ++i)
        InsertOctreeIndex(0, i, 0);

    m_stats.spatialCellsOrNodes = static_cast<int>(m_octreeNodes.size());
    size_t nodeMemory = m_octreeNodes.capacity() * sizeof(OctreeNode);
    for (const auto& node : m_octreeNodes)
        nodeMemory += node.indices.capacity() * sizeof(size_t);

    m_stats.memoryEstimateBytes =
        m_allBoids.capacity() * sizeof(BoidRef) +
        m_updateBoidIndices.capacity() * sizeof(size_t) +
        nodeMemory;
}

void FlockingSystem::InsertOctreeIndex(int nodeIndex, size_t boidIndex, int depth)
{
    if (m_octreeNodes[nodeIndex].leaf)
    {
        OctreeNode& node = m_octreeNodes[nodeIndex];
        if (node.indices.size() < kOctreeLeafCapacity || depth >= kOctreeMaxDepth)
        {
            node.indices.push_back(boidIndex);
            return;
        }

        std::vector<size_t> existing = std::move(node.indices);
        node.indices.clear();
        node.leaf = false;
        node.children.fill(-1);

        for (const size_t existingIndex : existing)
            InsertOctreeIndex(nodeIndex, existingIndex, depth);
    }

    OctreeNode& node = m_octreeNodes[nodeIndex];
    const glm::vec3 center = (node.min + node.max) * 0.5f;
    const glm::vec3 p = m_allBoids[boidIndex].position;
    int child = 0;
    if (p.x >= center.x) child |= 1;
    if (p.y >= center.y) child |= 2;
    if (p.z >= center.z) child |= 4;

    if (node.children[child] < 0)
    {
        OctreeNode childNode{};
        childNode.children.fill(-1);
        childNode.min = node.min;
        childNode.max = center;
        if (child & 1)
        {
            childNode.min.x = center.x;
            childNode.max.x = node.max.x;
        }
        if (child & 2)
        {
            childNode.min.y = center.y;
            childNode.max.y = node.max.y;
        }
        if (child & 4)
        {
            childNode.min.z = center.z;
            childNode.max.z = node.max.z;
        }

        const int childIndex = static_cast<int>(m_octreeNodes.size());
        node.children[child] = childIndex;
        m_octreeNodes.push_back(std::move(childNode));
        InsertOctreeIndex(childIndex, boidIndex, depth + 1);
        return;
    }

    InsertOctreeIndex(node.children[child], boidIndex, depth + 1);
}

void FlockingSystem::QueryNeighbours(
    World& world,
    const BoidRef& boid,
    const FlockingComponent& flock,
    std::vector<const BoidRef*>& perceptionNeighbours,
    std::vector<const BoidRef*>& separationNeighbours)
{
    if (m_searchMode == FlockingNeighbourSearchMode::UniformGrid)
    {
        QueryNeighboursUniformGrid(world, boid, flock, perceptionNeighbours, separationNeighbours);
    }
    else if (m_searchMode == FlockingNeighbourSearchMode::Octree)
    {
        QueryNeighboursOctree(world, boid, flock, perceptionNeighbours, separationNeighbours);
    }
    else
    {
        QueryNeighboursBruteForce(world, boid, flock, perceptionNeighbours, separationNeighbours);
    }
}

void FlockingSystem::QueryNeighboursBruteForce(
    World& world,
    const BoidRef& boid,
    const FlockingComponent& flock,
    std::vector<const BoidRef*>& perceptionNeighbours,
    std::vector<const BoidRef*>& separationNeighbours)
{
    for (size_t i = 0; i < m_allBoids.size(); ++i)
        TestNeighbourCandidate(world, boid, flock, i, perceptionNeighbours, separationNeighbours);
}

void FlockingSystem::QueryNeighboursUniformGrid(
    World& world,
    const BoidRef& boid,
    const FlockingComponent& flock,
    std::vector<const BoidRef*>& perceptionNeighbours,
    std::vector<const BoidRef*>& separationNeighbours)
{
    const float cellSize = 6.0f;
    const int range = std::max(1, static_cast<int>(std::ceil(RadiusForMode(flock) / cellSize)));
    const glm::vec3 p = boid.position / cellSize;
    const GridKey base{
        static_cast<int>(std::floor(p.x)),
        static_cast<int>(std::floor(p.y)),
        static_cast<int>(std::floor(p.z))
    };

    for (int z = -range; z <= range; ++z)
    {
        for (int y = -range; y <= range; ++y)
        {
            for (int x = -range; x <= range; ++x)
            {
                const GridKey key{ base.x + x, base.y + y, base.z + z };
                auto it = m_uniformGrid.find(key);
                if (it == m_uniformGrid.end())
                    continue;

                for (const size_t candidateIndex : it->second)
                {
                    ++m_stats.spatialCandidateChecks;
                    TestNeighbourCandidate(world, boid, flock, candidateIndex, perceptionNeighbours, separationNeighbours);
                }
            }
        }
    }
}

void FlockingSystem::QueryNeighboursOctree(
    World& world,
    const BoidRef& boid,
    const FlockingComponent& flock,
    std::vector<const BoidRef*>& perceptionNeighbours,
    std::vector<const BoidRef*>& separationNeighbours)
{
    if (m_octreeNodes.empty())
        return;

    std::vector<size_t> candidates;
    candidates.reserve(64);
    const float radius = RadiusForMode(flock);
    QueryOctreeNode(0, boid.position, radius * radius, candidates);

    for (const size_t candidateIndex : candidates)
    {
        ++m_stats.spatialCandidateChecks;
        TestNeighbourCandidate(world, boid, flock, candidateIndex, perceptionNeighbours, separationNeighbours);
    }
}

void FlockingSystem::QueryOctreeNode(
    int nodeIndex,
    const glm::vec3& center,
    float radiusSq,
    std::vector<size_t>& candidates) const
{
    const OctreeNode& node = m_octreeNodes[nodeIndex];
    if (!SphereIntersectsAabb(center, radiusSq, node.min, node.max))
        return;

    if (node.leaf)
    {
        candidates.insert(candidates.end(), node.indices.begin(), node.indices.end());
        return;
    }

    for (const int child : node.children)
    {
        if (child >= 0)
            QueryOctreeNode(child, center, radiusSq, candidates);
    }
}

void FlockingSystem::TestNeighbourCandidate(
    World& world,
    const BoidRef& boid,
    const FlockingComponent& flock,
    size_t candidateIndex,
    std::vector<const BoidRef*>& perceptionNeighbours,
    std::vector<const BoidRef*>& separationNeighbours)
{
    if (candidateIndex >= m_allBoids.size())
        return;

    const auto& other = m_allBoids[candidateIndex];
    if (other.entity == boid.entity)
        return;

    if (flock.flockId != 0)
    {
        const auto* otherFlock = world.getComponent<FlockingComponent>(other.entity);
        if (!otherFlock || otherFlock->flockId != flock.flockId)
            return;
    }

    ++m_stats.neighbourChecks;
    const glm::vec3 offset = other.position - boid.position;
    const float distSq = glm::dot(offset, offset);
    const float perceptionRadiusSq = flock.perceptionRadius * flock.perceptionRadius;
    const float separationRadiusSq = flock.separationRadius * flock.separationRadius;

    if (distSq < perceptionRadiusSq)
    {
        perceptionNeighbours.push_back(&other);
        ++m_stats.neighboursFound;
    }

    if (distSq < separationRadiusSq)
        separationNeighbours.push_back(&other);
}

glm::vec3 FlockingSystem::CalculateCohesion(
    const BoidRef& boid,
    const std::vector<const BoidRef*>& neighbours,
    float maxSpeed,
    float maxForce) const
{
    if (neighbours.empty())
        return glm::vec3(0.0f);

    glm::vec3 averagePosition{ 0.0f };
    for (const BoidRef* neighbour : neighbours)
        averagePosition += neighbour->position;

    averagePosition /= static_cast<float>(neighbours.size());
    const glm::vec3 desiredDirection = averagePosition - boid.position;
    if (glm::dot(desiredDirection, desiredDirection) < kEpsilon)
        return glm::vec3(0.0f);

    const glm::vec3 desiredVelocity = SafeNormalize(desiredDirection) * maxSpeed;
    return ClampLength(desiredVelocity - boid.velocity, maxForce);
}

glm::vec3 FlockingSystem::CalculateAlignment(
    const BoidRef& boid,
    const std::vector<const BoidRef*>& neighbours,
    float maxSpeed,
    float maxForce) const
{
    if (neighbours.empty())
        return glm::vec3(0.0f);

    glm::vec3 averageVelocity{ 0.0f };
    for (const BoidRef* neighbour : neighbours)
        averageVelocity += neighbour->velocity;

    averageVelocity /= static_cast<float>(neighbours.size());
    if (glm::dot(averageVelocity, averageVelocity) < kEpsilon)
        return glm::vec3(0.0f);

    const glm::vec3 desiredVelocity = SafeNormalize(averageVelocity) * maxSpeed;
    return ClampLength(desiredVelocity - boid.velocity, maxForce);
}

glm::vec3 FlockingSystem::CalculateSeparation(
    const BoidRef& boid,
    const std::vector<const BoidRef*>& neighbours,
    float maxSpeed,
    float maxForce) const
{
    if (neighbours.empty())
        return glm::vec3(0.0f);

    glm::vec3 separationSum{ 0.0f };
    int count = 0;
    for (const BoidRef* neighbour : neighbours)
    {
        const glm::vec3 away = boid.position - neighbour->position;
        const float distSq = glm::dot(away, away);
        if (distSq < kEpsilon)
            continue;

        const float distance = std::sqrt(distSq);
        separationSum += (away / distance) / distance;
        ++count;
    }

    if (count == 0 || glm::dot(separationSum, separationSum) < kEpsilon)
        return glm::vec3(0.0f);

    const glm::vec3 desiredVelocity = SafeNormalize(separationSum) * maxSpeed;
    return ClampLength(desiredVelocity - boid.velocity, maxForce);
}

glm::vec3 FlockingSystem::CalculateAvoidance(
    const BoidRef& boid,
    float maxSpeed,
    float maxForce)
{
    glm::vec3 avoidance{ 0.0f };

    if (m_boundsEnabled)
    {
        auto addBoundsAxis = [&](float value, float minV, float maxV, const glm::vec3& axis)
            {
                if (value < minV + m_boundsMargin)
                {
                    const float t = 1.0f - std::max(0.0f, value - minV) / std::max(kEpsilon, m_boundsMargin);
                    avoidance += axis * (t * m_boundsAvoidanceStrength);
                }
                else if (value > maxV - m_boundsMargin)
                {
                    const float t = 1.0f - std::max(0.0f, maxV - value) / std::max(kEpsilon, m_boundsMargin);
                    avoidance -= axis * (t * m_boundsAvoidanceStrength);
                }
            };

        addBoundsAxis(boid.position.x, m_boundsMin.x, m_boundsMax.x, glm::vec3(1, 0, 0));
        addBoundsAxis(boid.position.y, m_boundsMin.y, m_boundsMax.y, glm::vec3(0, 1, 0));
        addBoundsAxis(boid.position.z, m_boundsMin.z, m_boundsMax.z, glm::vec3(0, 0, 1));
    }

    const glm::vec3 forward = SafeNormalize(boid.velocity, glm::vec3(0, 0, 1));
    const glm::vec3 lookAheadPosition = boid.position + forward * m_lookAheadDistance;

    for (const auto& obstacle : m_obstacles)
    {
        if (!obstacle.enabled)
            continue;

        ++m_stats.collisionAvoidanceChecks;

        if (obstacle.type == FlockingObstacleType::Sphere)
        {
            const glm::vec3 away = lookAheadPosition - obstacle.position;
            const float distance = glm::length(away);
            const float triggerDistance = obstacle.radius + boid.boidRadius + m_obstacleMargin;
            if (distance < triggerDistance)
            {
                const float t = 1.0f - distance / std::max(kEpsilon, triggerDistance);
                avoidance += SafeNormalize(away, -forward) * (m_boundsAvoidanceStrength * t);
            }
        }
        else if (obstacle.type == FlockingObstacleType::Aabb)
        {
            const glm::vec3 closest = glm::clamp(lookAheadPosition, obstacle.min, obstacle.max);
            const glm::vec3 away = lookAheadPosition - closest;
            const float distance = glm::length(away);
            const float triggerDistance = boid.boidRadius + m_obstacleMargin;
            if (distance < triggerDistance)
            {
                const float t = 1.0f - distance / std::max(kEpsilon, triggerDistance);
                avoidance += SafeNormalize(away, -forward) * (m_boundsAvoidanceStrength * t);
            }
        }
        else if (obstacle.type == FlockingObstacleType::Plane)
        {
            const float currentDistance = glm::dot(boid.position - obstacle.position, obstacle.normal);
            const float futureDistance = glm::dot(lookAheadPosition - obstacle.position, obstacle.normal);
            const float distance = std::min(currentDistance, futureDistance);
            if (distance < m_obstacleMargin)
            {
                const float t = 1.0f - distance / std::max(kEpsilon, m_obstacleMargin);
                avoidance += obstacle.normal * (m_boundsAvoidanceStrength * t);
            }
        }
    }

    if (glm::dot(avoidance, avoidance) < kEpsilon)
        return glm::vec3(0.0f);

    const glm::vec3 desiredVelocity = SafeNormalize(avoidance) * maxSpeed;
    return ClampLength(desiredVelocity - boid.velocity, maxForce);
}

glm::vec3 FlockingSystem::ClampLength(const glm::vec3& v, float maxLength)
{
    if (maxLength <= 0.0f)
        return glm::vec3(0.0f);

    const float lenSq = glm::dot(v, v);
    const float maxSq = maxLength * maxLength;
    if (lenSq <= maxSq || lenSq < kEpsilon)
        return v;

    return v * (maxLength / std::sqrt(lenSq));
}

glm::vec3 FlockingSystem::SafeNormalize(const glm::vec3& v, const glm::vec3& fallback)
{
    const float lenSq = glm::dot(v, v);
    if (lenSq < kEpsilon)
        return fallback;

    return v / std::sqrt(lenSq);
}

void FlockingSystem::WeightedTruncatedAdd(glm::vec3& total, float& remaining, const glm::vec3& force)
{
    if (remaining <= 0.0f)
        return;

    const float lenSq = glm::dot(force, force);
    if (lenSq < kEpsilon)
        return;

    const float len = std::sqrt(lenSq);
    if (len <= remaining)
    {
        total += force;
        remaining -= len;
        return;
    }

    total += (force / len) * remaining;
    remaining = 0.0f;
}

bool FlockingSystem::ShouldUpdateEntity(World& world, Entity e) const
{
    const auto* owner = world.getComponent<OwnerComponent>(e);
    if (!owner || owner->ownerId < 0 || m_localPeerId <= 0)
        return true;

    return owner->ownerId == (m_localPeerId - 1);
}
