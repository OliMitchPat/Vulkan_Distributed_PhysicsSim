#include "ReplicaSmoothing.h"

#include <algorithm>
#include <chrono>
#include <cmath>

void ReplicaSnapshotRing::Clear()
{
    head = 0;
    count = 0;
}

bool ReplicaSnapshotRing::Empty() const
{
    return count == 0;
}

const ReplicaState& ReplicaSnapshotRing::At(size_t idx) const
{
    return samples[(head + idx) % kReplicaSnapshotRingCapacity];
}

const ReplicaState& ReplicaSnapshotRing::Latest() const
{
    return At(count - 1);
}

void ReplicaSnapshotRing::Push(const ReplicaState& s)
{
    if (count > 0)
    {
        const ReplicaState& latest = Latest();
        if (s.tick <= latest.tick)
            return;
    }

    const size_t insertIdx = (head + count) % kReplicaSnapshotRingCapacity;
    samples[insertIdx] = s;

    if (count < kReplicaSnapshotRingCapacity)
    {
        ++count;
    }
    else
    {
        head = (head + 1) % kReplicaSnapshotRingCapacity;
    }
}

double NowSeconds()
{
    using clock = std::chrono::steady_clock;
    const auto now = clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
}

glm::quat IntegrateOrientation(const glm::quat& rot, const glm::vec3& angVel, float dt)
{
    const float speed = glm::length(angVel);
    if (speed <= 1e-5f || dt <= 0.0f)
        return rot;

    const glm::vec3 axis = angVel / speed;
    const float angle = speed * dt;
    const glm::quat delta = glm::angleAxis(angle, axis);
    return glm::normalize(delta * rot);
}

bool SampleReplicaStateAtTime(
    const ReplicaSnapshotRing& ring,
    double sampleTimeSec,
    float maxExtrapSec,
    ReplicaState& outState,
    bool& outUsedExtrapolation)
{
    outUsedExtrapolation = false;
    if (ring.Empty())
        return false;

    if (ring.count == 1)
    {
        outState = ring.Latest();
        return true;
    }

    const ReplicaState& oldest = ring.At(0);
    const ReplicaState& newest = ring.Latest();

    if (sampleTimeSec <= oldest.recvTimeSec)
    {
        outState = oldest;
        return true;
    }

    for (size_t i = 1; i < ring.count; ++i)
    {
        const ReplicaState& b = ring.At(i);
        if (sampleTimeSec <= b.recvTimeSec)
        {
            const ReplicaState& a = ring.At(i - 1);
            const double span = std::max(1e-5, b.recvTimeSec - a.recvTimeSec);
            const float t = (float)((sampleTimeSec - a.recvTimeSec) / span);
            const float clampedT = glm::clamp(t, 0.0f, 1.0f);

            outState = a;
            outState.pos = glm::mix(a.pos, b.pos, clampedT);
            outState.rot = glm::normalize(glm::slerp(a.rot, b.rot, clampedT));
            outState.linVel = glm::mix(a.linVel, b.linVel, clampedT);
            outState.angVel = glm::mix(a.angVel, b.angVel, clampedT);
            outState.tick = (clampedT < 0.5f) ? a.tick : b.tick;
            outState.recvTimeSec = sampleTimeSec;
            return true;
        }
    }

    outUsedExtrapolation = true;
    outState = newest;

    const double dtSec = std::max(0.0, sampleTimeSec - newest.recvTimeSec);
    const float clampedExtrap = std::min((float)dtSec, maxExtrapSec);
    outState.pos = newest.pos + newest.linVel * clampedExtrap;
    outState.rot = IntegrateOrientation(newest.rot, newest.angVel, clampedExtrap);
    outState.recvTimeSec = newest.recvTimeSec + clampedExtrap;
    return true;
}
