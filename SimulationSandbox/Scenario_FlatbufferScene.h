#pragma once

#include "Scenario.h"
#include "NetProtocol.h"
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// Forward declarations
namespace SimIO { class SceneLoader; }

// -----------------------------------------------------------------------
// Scenario_FlatbufferScene
//
// Loads a FlatBuffers .bin scene file and spawns entities into the World.
// Entities are created in the exact order of scene->objects() so that
// entity IDs are identical across all peers (0 = object index 0, etc.).
// -----------------------------------------------------------------------
class Scenario_FlatbufferScene final : public Scenario
{
public:
    // binPath is relative to the working directory (e.g. "assets/scenes/newtonsCradle.bin")
    explicit Scenario_FlatbufferScene(const std::string& binPath);
    ~Scenario_FlatbufferScene() override;

    const char* Name()      const override;
    bool        GravityOn() const override { return m_gravityOn; }

    void OnLoad  (World& world) override;
    void OnUnload(World& world) override;
    void Update(World& world, float dt, uint32_t currentSceneGeneration) override;

    void SetLocalPeerId(int peerId);
    void SetConfiguredPeerIds(const std::vector<int>& peerIds);
    bool HasFlockingAgents() const { return m_hasFlockingAgents; }
    void RequestFlockingBoidCount(int count);
    size_t PendingSpawnCount() const { return m_pendingSpawnEvents.size(); }
    bool PopPendingSpawn(Net::SpawnObjectPayload& outPayload);
    bool SpawnFromNetworkEvent(World& world, const Net::SpawnObjectPayload& payload);

private:
    struct SpawnerRuntime
    {
        uint32_t emittedCount = 0;
        uint32_t spawnCounter = 0;
        float nextSpawnTimeSec = 0.0f;
        bool burstDone = false;
    };

    void ApplyRuntimeFlockingBoidCount(World& world, int targetCount);

    std::string m_path;
    std::string m_displayName;   // set after first successful load
    bool        m_gravityOn = true;
    bool        m_hasFlockingAgents = false;
    int         m_localPeerId = 1; // 1..4
    std::vector<int> m_configuredPeerIds{ 1 };
    std::atomic<int> m_requestedFlockingBoidCount{ -1 };
    int         m_appliedFlockingBoidCount = -1;
    float       m_spawnerElapsedSec = 0.0f;
    float       m_nextSpawnerWakeSec = 0.0f;
    std::vector<SpawnerRuntime> m_spawnerRuntime;
    std::vector<Net::SpawnObjectPayload> m_pendingSpawnEvents;

    std::unique_ptr<SimIO::SceneLoader> m_loader;
};
