#pragma once

#include <glm/glm.hpp>
#include <string>
#include <vector>
#include "World.h"
#include "ShapeData.h"
#include "RigidBody.h"  

// --------------------------------------------------
// Enums used by multiple components
// --------------------------------------------------
struct ParentComponent
{
    Entity parent = INVALID_ENTITY;
};


enum class ShadingModel
{
    Gouraud,   // per-vertex shading
    Phong      // per-pixel shading
};

enum class CameraRole
{
    Overview,        // C1 - outside view of globe
    Navigation,      // C2 - user navigation near particles/shadows
    CloseUp    // C3 - focused on a cactus
};

enum class WeatherType
{
    Clear,
    Rain,
    Snow,
    Sandstorm
};

enum class ParticleType
{
    Fire,
    Snow,
    Rain,
    Dust,
    Spark
};

struct OwnerComponent
{
    int ownerId = -1; // -1 = owned by all / local copy
};


//--------------------------------------------------    
//Animated Components
// --------------------------------------------------

enum class AnimatedEasing
{
    Linear,
    Smoothstep
};

enum class AnimatedPathMode
{
    Stop,
    Loop,
    Reverse
};

struct AnimatedWaypoint
{
    glm::vec3 position{ 0.0f };
    glm::quat rotation{ 1.0f, 0.0f, 0.0f, 0.0f };
    float time = 0.0f; // absolute time from animation start
};

struct AnimatedPathComponent
{
    std::vector<AnimatedWaypoint> waypoints;

    float totalDuration = 0.0f;
    float elapsed = 0.0f;

    AnimatedEasing easing = AnimatedEasing::Linear;
    AnimatedPathMode mode = AnimatedPathMode::Stop;
};

// --------------------------------------------------
// PhysicsComponent
// --------------------------------------------------
struct PhysicsComponent
{
    RigidBody body;

    float density = 1000.0f;
    BodyMotionType behaviour = BodyMotionType::Dynamic;

    float restitution = 0.5f;
    float staticFriction = 0.5f;
    float dynamicFriction = 0.3f;

    bool initialized = false;
};

struct SphereColliderComponent
{
    float baseRadius = 0.5f;
    glm::vec3 localCenter{ 0.0f };
};

struct PlaneColliderComponent
{
    glm::vec3 normal{ 0,1,0 };   // should be normalized
    float offset = 0.0f;       // plane equation: dot(n, x) - offset = 0
    // alternatively store a point on plane instead of offset:
    // glm::vec3 point{0,0,0};
};

struct CuboidColliderComponent
{
    glm::vec3 halfExtents{ 0.5f, 0.5f, 0.5f };  // half-size in each axis (baseSize / 2)
    glm::vec3 localCenter{ 0.0f };               // offset of box center from entity origin
};

struct CylinderColliderComponent
{
    float     radius = 0.5f;       // baseRadius
    float     height = 1.0f;       // baseHeight; full height, aligned to local Y axis
    glm::vec3 localCenter{ 0.0f }; // offset of cylinder midpoint from entity origin
};

struct CapsuleColliderComponent
{
    float     radius = 0.5f;       // baseRadius
    float     height = 1.0f;       // baseHeight; full height of the cylindrical segment, local Y
    glm::vec3 localCenter{ 0.0f }; // offset of capsule midpoint from entity origin
};

// --------------------------------------------------
// ShapeComponent
// --------------------------------------------------
//
// Generic shape component that wraps a ShapeData variant.
// Preferred over the individual collider components when
// loading objects from FlatBuffers scenes.
//
struct ShapeComponent
{
    ShapeData     shape;
    CollisionType collisionType = CollisionType::SOLID;
};

// --------------------------------------------------
// NameComponent
// --------------------------------------------------
//
// Stores the resolved display name for an entity.
// Objects without a name in the FlatBuffer get an
// auto-generated name such as "object 0".
//
struct NameComponent
{
    std::string name;

    NameComponent() = default;
    explicit NameComponent(std::string n) : name(std::move(n)) {}
};

// --------------------------------------------------
// TransformComponent
// --------------------------------------------------
//
// Basic spatial transform for any entity in the world.
//
struct TransformComponent
{
    glm::vec3 position{ 0.0f };
    glm::vec3 rotation{ 0.0f }; // Euler angles in radians (pitch, yaw, roll)
    glm::vec3 scale{ 1.0f };

    TransformComponent() = default;

    explicit TransformComponent(const glm::vec3& pos)
        : position(pos)
    {
    }

    TransformComponent(const glm::vec3& pos,
        const glm::vec3& rot,
        const glm::vec3& scl)
        : position(pos), rotation(rot), scale(scl)
    {
    }
};

// --------------------------------------------------
// VelocityComponent
// --------------------------------------------------
//
// Optional movement data for animated entities
// (e.g. animals, moving cameras, drifting particles).
//
struct VelocityComponent
{
    glm::vec3 linearVelocity{ 0.0f };
    glm::vec3 angularVelocity{ 0.0f };

    VelocityComponent() = default;

    explicit VelocityComponent(const glm::vec3& linVel,
        const glm::vec3& angVel = glm::vec3{ 0.0f })
        : linearVelocity(linVel), angularVelocity(angVel)
    {
    }
};

// --------------------------------------------------
// RenderMeshComponent
// --------------------------------------------------
//
// Links an entity to mesh/texture resources. This is a
// lightweight description; actual GPU buffers are owned
// by the renderer / resource manager.
//
struct RenderMeshComponent
{
    std::string meshName;      // e.g. "cactus.obj"
    std::string textureName;   // e.g. "cactus_diffuse.png"

    RenderMeshComponent() = default;

    RenderMeshComponent(const std::string& mesh,
        const std::string& texture)
        : meshName(mesh), textureName(texture)
    {
    }
};

// --------------------------------------------------
// MaterialComponent
// --------------------------------------------------
//
// Controls shading model and basic material properties.
// This will be useful to demonstrate Gouraud vs Phong
// and to tweak how shiny / coloured objects are.
//
struct MaterialComponent
{
    ShadingModel shadingModel = ShadingModel::Phong;

    glm::vec3 diffuseColor{ 1.0f, 1.0f, 1.0f };
    glm::vec3 specularColor{ 1.0f, 1.0f, 1.0f };
    float shininess = 32.0f;

    // 1.0 = opaque, 0.0 = fully transparent
    float alpha = 1.0f;

    bool castsShadows = true;
    bool receivesShadows = true;

    MaterialComponent() = default;

    MaterialComponent(ShadingModel model,
        const glm::vec3& diffuse,
        const glm::vec3& specular,
        float shininessValue,
        float alphaValue = 1.0f)
        : shadingModel(model),
        diffuseColor(diffuse),
        specularColor(specular),
        shininess(shininessValue),
        alpha(alphaValue)
    {
    }
};

// --------------------------------------------------
// CameraComponent
// --------------------------------------------------
//
// Basic camera parameters. Combined with a TransformComponent
// to build a full view/projection matrix.
//
struct CameraComponent
{
    float fov = 60.0f;      // degrees, will be converted to radians
    float nearClip = 0.1f;
    float farClip = 250.0f;

    CameraComponent() = default;

    CameraComponent(float fovDegrees,
        float nearPlane,
        float farPlane)
        : fov(fovDegrees), nearClip(nearPlane), farClip(farPlane)
    {
    }
};

// --------------------------------------------------
// CameraRoleComponent
// --------------------------------------------------
//
// Indicates which logical camera this is (C1, C2, C3).
// This makes it easy to switch on F1/F2/F3.
//
struct CameraRoleComponent
{
    CameraRole role = CameraRole::Overview;

    CameraRoleComponent() = default;

    explicit CameraRoleComponent(CameraRole r)
        : role(r)
    {
    }
};

// --------------------------------------------------
// EnvironmentStateComponent
// --------------------------------------------------
//
// Singleton-style component (attached to one "environment"
// entity) that stores global simulation state like time of
// day, season, and weather conditions.
//
struct EnvironmentStateComponent
{
    float temperature = 20.0f;         // degrees Celsius
    float sandstormStrength = 0.0f;    // 0..1
    float dayNightTime = 0.0f;         // 0..1 (progress through the day)
    float seasonTime = 0.0f;           // 0..1 (progress through the season)
    float worldTime = 0.0f;
    WeatherType weather = WeatherType::Clear;

    // Time scaling controlled by user (t/T keys)
    float timeScale = 1.0f;

    bool sandstormActive = false;

    glm::vec3 sandstormPos = glm::vec3(0.0f);  // current storm centre
    glm::vec3 sandstormDir = glm::vec3(1.0f, 0.0f, 0.0f); // movement direction (XZ)

    float sandstormSpeed = 8.0f;

    EnvironmentStateComponent() = default;
};

// --------------------------------------------------
// CactusComponent
// --------------------------------------------------
//
// Simulation data for cactus behaviour. Systems will read
// EnvironmentStateComponent and update these fields over time.
//
struct CactusComponent
{
    float age = 0.0f;         // seconds or arbitrary units
    float growth = 1.0f;      // used to scale the cactus mesh
    float hydration = 1.0f;   // 0..1, affects growth and burning
    float health = 1.0f;      // 0..1

    bool burning = false;
    float burnTimer = 0.0f;   // time since burning started
    bool hasFireEmitter = false;    
    CactusComponent() = default;
};


// --------------------------------------------------
// DirectionalLightComponent
// --------------------------------------------------
//
// For sun and moon entities. Direction is in world space.
// Intensity and colour drive lighting & shadow mapping.
//
struct DirectionalLightComponent
{
    glm::vec3 direction{ 0.0f, -1.0f, 0.0f }; // pointing down by default
    glm::vec3 color{ 1.0f, 1.0f, 1.0f };
    float intensity = 1.0f;

    bool castsShadows = true;

    DirectionalLightComponent() = default;

    DirectionalLightComponent(const glm::vec3& dir,
        const glm::vec3& col,
        float intens,
        bool casts = true)
        : direction(dir), color(col), intensity(intens), castsShadows(casts)
    {
    }
};

// --------------------------------------------------
// ParticleEmitterComponent
// --------------------------------------------------
//
// Used for fire, snow, rain, dust, etc. The actual particle
// system will read these settings and spawn/simulate particles.
//
struct ParticleEmitterComponent
{
    ParticleType type = ParticleType::Fire;

    float emissionRate = 50.0f;      // particles per second
    float particleLifetime = 2.0f;   // seconds

    glm::vec3 initialVelocity{ 0.0f, 1.0f, 0.0f };
    glm::vec3 velocityRandomness{ 0.5f, 0.5f, 0.5f };

    bool active = true;

    bool followCamera = false;   
    glm::vec3 followOffset = { 0, 10, 0 }; 

    bool  stormInit = false;
    glm::vec3 stormDir = glm::vec3(1, 0, 0); // unit dir in XZ
    float stormSpeed = 8.0f;                // units/sec
    float stormTime = 0.0f;                // for wobble phase
    float cloudRadius = 12.0f;

    ParticleEmitterComponent() = default;

    ParticleEmitterComponent(ParticleType t,
        float rate,
        float lifetime)
        : type(t), emissionRate(rate), particleLifetime(lifetime)
    {
    }
};

struct SparkLightComponent
{
    glm::vec3 color = glm::vec3(1.0f, 0.7f, 0.2f);
    float intensity = 5.0f;
    float radius = 3.0f;
    bool active = true;
};


struct SimParticle
{
    ParticleType type = ParticleType::Dust;
    glm::vec3 position{ 0.0f };
    glm::vec3 velocity{ 0.0f };
    float age = 0.0f;
    float lifetime = 1.0f;
};

struct ParticlePoolComponent
{
    std::vector<SimParticle> particles;
    size_t maxParticles = 10000; // safety cap

    ParticlePoolComponent() = default;
};
