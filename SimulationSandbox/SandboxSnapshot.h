#pragma once

#include "WorldSnapshot.h"

#include <glm/glm.hpp>
#include <cstdint>
#include <memory>

class World;

const char* DisplayModeName(int mode);

std::shared_ptr<WorldSnapshot> CaptureSnapshot(
    World& world,
    uint64_t tickNumber,
    int displayMode,
    const glm::vec4& clearColor);
