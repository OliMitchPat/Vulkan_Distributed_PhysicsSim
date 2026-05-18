#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cstdint>

struct ReplicaState
{
    uint32_t objectId = 0;
    glm::vec3 pos{ 0 };
    glm::quat rot{ 1, 0, 0, 0 };
    glm::vec3 linVel{ 0 };
    glm::vec3 angVel{ 0 };
    uint32_t tick = 0;
    double recvTimeSec = 0.0;
};

static constexpr size_t kReplicaSnapshotRingCapacity = 64;

struct ReplicaSnapshotRing
{
    std::array<ReplicaState, kReplicaSnapshotRingCapacity> samples{};
    size_t head = 0;
    size_t count = 0;

    void Clear();
    bool Empty() const;
    const ReplicaState& At(size_t idx) const;
    const ReplicaState& Latest() const;
    void Push(const ReplicaState& s);
};

double NowSeconds();

glm::quat IntegrateOrientation(const glm::quat& rot, const glm::vec3& angVel, float dt);

bool SampleReplicaStateAtTime(
    const ReplicaSnapshotRing& ring,
    double sampleTimeSec,
    float maxExtrapSec,
    ReplicaState& outState,
    bool& outUsedExtrapolation);
