#pragma once

#include "Scenario.h"
#include <memory>
#include <string>

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

private:
    std::string m_path;
    std::string m_displayName;   // set after first successful load
    bool        m_gravityOn = true;

    std::unique_ptr<SimIO::SceneLoader> m_loader;
};
