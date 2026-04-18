#pragma once
#include <memory>
#include <vector>

class Scenario;
class World;

class ScenarioManager
{
public:
    void Add(std::unique_ptr<Scenario> scenario);
    Scenario* Current() const { return current; }

    void SwitchTo(World& world, int index);

    int Count() const { return (int)scenarios.size(); }
    Scenario* Get(int i) const { return scenarios[i].get(); }
    int CurrentIndex() const { return currentIndex; }

private:
    std::vector<std::unique_ptr<Scenario>> scenarios;
    Scenario* current = nullptr;
    int currentIndex = -1;
};
