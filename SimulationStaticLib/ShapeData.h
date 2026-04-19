#pragma once
#include <glm/glm.hpp>
#include <variant>

struct SphereShape { float radius = 0.5f; };
struct CuboidShape { glm::vec3 size{ 1.0f }; };                 // full extents
struct CylinderShape { float radius = 0.5f; float height = 1.0f; }; // local Y
struct CapsuleShape { float radius = 0.5f; float height = 1.0f; }; // local Y
struct PlaneShape { glm::vec3 normal{ 0,1,0 }; };              // local-space normal

using ShapeData = std::variant<SphereShape, PlaneShape, CapsuleShape, CylinderShape, CuboidShape>;

enum class CollisionType { SOLID, CONTAINER };
enum class BehaviourType { Static, Animated, Simulated };