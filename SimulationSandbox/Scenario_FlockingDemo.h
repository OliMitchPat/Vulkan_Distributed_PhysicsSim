#pragma once

#include "Scenario.h"
#include <atomic>

class World;

class Scenario_FlockingDemo final : public Scenario
{
public:
    const char* Name() const override { return "Advanced Flocking Demo"; }
    bool GravityOn() const override { return false; }

    void OnLoad(World& world) override;
    void Update(World& world, float dt, uint32_t currentSceneGeneration) override;

    void RequestBoidCount(int count);
    int CurrentBoidCount() const { return m_currentBoidCount; }

private:
    void CreateManualFlockingScene(World& world, int boidCount);

private:
    std::atomic<int> m_requestedBoidCount{ 100 };
    int m_currentBoidCount = 0;
};
