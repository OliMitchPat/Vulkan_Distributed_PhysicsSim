#pragma once
#include "Scenario.h"

class Scenario_PrimitiveScene final : public Scenario
{
public:
    const char* Name() const override { return "Primitive Scene"; }
    void OnLoad(World& world) override;
};