#include "ScenarioManager.h"
#include "Scenario.h"
#include "World.h"

void ScenarioManager::Add(std::unique_ptr<Scenario> scenario)
{
    scenarios.push_back(std::move(scenario));
}

void ScenarioManager::SwitchTo(World& world, int index)
{
    if (index < 0 || index >= (int)scenarios.size()) return;
    if (index == currentIndex) return;

    if (current)
        current->OnUnload(world);

    world.Clear();

    currentIndex = index;
    current = scenarios[index].get();
    current->OnLoad(world);
}
