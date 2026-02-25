#pragma once

#include <cstdint>
#include <cstddef>

using Entity = std::uint32_t;

constexpr Entity INVALID_ENTITY = static_cast<Entity>(-1);

// Hard cap for simplicity; more than enough for this coursework.
constexpr std::size_t MAX_ENTITIES = 2048;

