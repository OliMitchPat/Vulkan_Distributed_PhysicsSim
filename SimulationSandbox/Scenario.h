#pragma once
#include <string>

class World;
class Renderer;

class Scenario
{
public:
    virtual ~Scenario() = default;

    virtual const char* Name() const = 0;

    virtual void OnLoad(World& world) {}
    virtual void OnUnload(World& world) {}

    virtual void Update(World& world, float dt) {}
    virtual void ImGuiMainMenu(World& world) {}  
};
