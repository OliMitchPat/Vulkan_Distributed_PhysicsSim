/*
 * Scenario_FlatbufferScene.cpp
 *
 * Loads a FlatBuffers .bin file (Scene.fbs schema) and spawns physics
 * entities into the World in the same order as scene->objects(), giving
 * each entity a deterministic ID matching its object index.
 *
 * Scale policy (Option A from spec):
 *   uniformScale = max(|sx|, |sy|, |sz|),  safe default 1.
 *
 * Euler rotation order (documented in FlatbufferConvert.h):
 *   yaw(Y) -> pitch(X) -> roll(Z)
 *
 * Fallback material constants (used when no matching interaction found):
 *   density      = 1000 kg/m^3
 *   restitution  = 0.5
 *   static_fric  = 0.5
 *   dynamic_fric = 0.3
 */
#include "Scenario_FlatbufferScene.h"
#include "World.h"
#include "Components.h"

// SimulationStaticLib headers
#include "SceneLoader.h"
#include "FlatbufferConvert.h"
#include "SetUpRigidBodyFromFlatbuffer.h"
#include "ShapeData.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_map>
#include <cstring>

// ---- Fallback material constants ----------------------------------------
namespace
{
    constexpr float kFallbackDensity      = 1000.0f;
    constexpr float kFallbackRestitution  = 0.5f;
    constexpr float kFallbackStaticFric   = 0.5f;
    constexpr float kFallbackDynamicFric  = 0.3f;

    // Per-object physics properties resolved from scene materials/interactions
    struct ResolvedMaterial
    {
        float density      = kFallbackDensity;
        float restitution  = kFallbackRestitution;
        float staticFric   = kFallbackStaticFric;
        float dynamicFric  = kFallbackDynamicFric;
    };

    // Returns a mesh name appropriate for a given ShapeData variant.
    // Keeps existing mesh names used by Scenario_PrimitiveScene.
    std::string MeshNameForShape(const ShapeData& shape)
    {
        return std::visit([](auto&& s) -> std::string
        {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, SphereShape>)   return "sphere.obj";
            if constexpr (std::is_same_v<T, CuboidShape>)   return "cube.obj";
            if constexpr (std::is_same_v<T, CylinderShape>) return "cylinder.obj";
            if constexpr (std::is_same_v<T, CapsuleShape>)  return "capsule.obj";
            if constexpr (std::is_same_v<T, PlaneShape>)    return "Plane.obj";
            return "sphere.obj";
        }, shape);
    }

    // Short string describing a shape for the scene dump log
    std::string ShapeDumpStr(const ShapeData& shape, float uniformScale)
    {
        return std::visit([uniformScale](auto&& s) -> std::string
        {
            using T = std::decay_t<decltype(s)>;
            char buf[128];
            if constexpr (std::is_same_v<T, SphereShape>)
            {
                std::snprintf(buf, sizeof(buf),
                    "Sphere  r=%.3f  scale=%.3f", s.radius, uniformScale);
            }
            else if constexpr (std::is_same_v<T, CuboidShape>)
            {
                std::snprintf(buf, sizeof(buf),
                    "Cuboid  size=(%.3f,%.3f,%.3f)  scale=%.3f",
                    s.size.x, s.size.y, s.size.z, uniformScale);
            }
            else if constexpr (std::is_same_v<T, CylinderShape>)
            {
                std::snprintf(buf, sizeof(buf),
                    "Cylinder  r=%.3f h=%.3f  scale=%.3f",
                    s.radius, s.height, uniformScale);
            }
            else if constexpr (std::is_same_v<T, CapsuleShape>)
            {
                std::snprintf(buf, sizeof(buf),
                    "Capsule  r=%.3f h=%.3f  scale=%.3f",
                    s.radius, s.height, uniformScale);
            }
            else if constexpr (std::is_same_v<T, PlaneShape>)
            {
                std::snprintf(buf, sizeof(buf),
                    "Plane  n=(%.3f,%.3f,%.3f)  scale=%.3f",
                    s.normal.x, s.normal.y, s.normal.z, uniformScale);
            }
            else
            {
                std::snprintf(buf, sizeof(buf), "Unknown  scale=%.3f", uniformScale);
            }
            return std::string(buf);
        }, shape);
    }

    // Build a density lookup map from scene materials
    std::unordered_map<std::string, float>
    BuildDensityMap(const Simulation::Scene* scene)
    {
        std::unordered_map<std::string, float> map;
        if (!scene || !scene->materials()) return map;

        for (unsigned i = 0; i < scene->materials()->size(); ++i)
        {
            const auto* mat = scene->materials()->Get(i);
            if (!mat) continue;
            const std::string name = SimIO::ToStdStringOrEmpty(mat->name());
            if (!name.empty())
                map[name] = std::max(0.0f, mat->density());
        }
        return map;
    }

    // Look up density by material name; returns fallback if not found.
    float LookupDensity(
        const std::unordered_map<std::string, float>& densityMap,
        const std::string& materialName)
    {
        if (!materialName.empty())
        {
            auto it = densityMap.find(materialName);
            if (it != densityMap.end())
                return it->second;
        }
        return kFallbackDensity;
    }

    // Look up material interaction constants for a given material name.
    // Searches interactions where material_a OR material_b matches.
    // Returns fallback constants if no match found.
    ResolvedMaterial LookupMaterialInteraction(
        const Simulation::Scene* scene,
        const std::string& materialName)
    {
        ResolvedMaterial result{};
        if (!scene || !scene->interactions() || materialName.empty())
            return result;

        for (unsigned i = 0; i < scene->interactions()->size(); ++i)
        {
            const auto* inter = scene->interactions()->Get(i);
            if (!inter) continue;

            const std::string a = SimIO::ToStdStringOrEmpty(inter->material_a());
            const std::string b = SimIO::ToStdStringOrEmpty(inter->material_b());

            if (a == materialName || b == materialName)
            {
                result.restitution = inter->restitution();
                result.staticFric  = inter->static_friction();
                result.dynamicFric = inter->dynamic_friction();
                return result;
            }
        }
        return result;
    }

    // String helpers for scene dump
    const char* BehaviourStr(Simulation::Behaviour b)
    {
        switch (b)
        {
        case Simulation::Behaviour_StaticObject:    return "Static";
        case Simulation::Behaviour_AnimatedObject:  return "Animated";
        case Simulation::Behaviour_SimulatedObject: return "Simulated";
        default:                                    return "Static";
        }
    }

    const char* CollisionTypeStr(Simulation::CollisionType c)
    {
        return (c == Simulation::CollisionType_CONTAINER) ? "CONTAINER" : "SOLID";
    }

    const char* ShapeTypeStr(Simulation::Shape s)
    {
        switch (s)
        {
        case Simulation::Shape_Sphere:   return "Sphere";
        case Simulation::Shape_Cuboid:   return "Cuboid";
        case Simulation::Shape_Cylinder: return "Cylinder";
        case Simulation::Shape_Capsule:  return "Capsule";
        case Simulation::Shape_Plane:    return "Plane";
        default:                         return "Unknown";
        }
    }

    static uint32_t Hash32(uint32_t x)
    {
        x ^= x >> 16;
        x *= 0x7feb352dU;
        x ^= x >> 15;
        x *= 0x846ca68bU;
        x ^= x >> 16;
        return x;
    }

    static float HashUnitFloat(uint32_t seed, uint32_t salt)
    {
        const uint32_t h = Hash32(seed ^ (salt * 0x9E3779B9u));
        return static_cast<float>(h & 0x00FFFFFFu) / static_cast<float>(0x01000000u);
    }

    static float SampleFloatRange(const Simulation::FloatRange* range, float fallback, uint32_t seed, uint32_t salt)
    {
        if (!range) return fallback;
        const float a = range->min();
        const float b = range->max();
        const float lo = std::min(a, b);
        const float hi = std::max(a, b);
        if (std::abs(hi - lo) <= 1e-6f) return lo;
        return lo + (hi - lo) * HashUnitFloat(seed, salt);
    }

    static glm::vec3 SampleVec3Range(const Simulation::Vec3Range* range, const glm::vec3& fallback, uint32_t seed, uint32_t saltBase)
    {
        if (!range) return fallback;
        const auto& mn = range->min();
        const auto& mx = range->max();
        const float x = std::min(mn.x(), mx.x()) + (std::max(mn.x(), mx.x()) - std::min(mn.x(), mx.x())) * HashUnitFloat(seed, saltBase + 0);
        const float y = std::min(mn.y(), mx.y()) + (std::max(mn.y(), mx.y()) - std::min(mn.y(), mx.y())) * HashUnitFloat(seed, saltBase + 1);
        const float z = std::min(mn.z(), mx.z()) + (std::max(mn.z(), mx.z()) - std::min(mn.z(), mx.z())) * HashUnitFloat(seed, saltBase + 2);
        return glm::vec3(x, y, z);
    }

    static int ResolveControllerPeerId(const Simulation::SpawnerOwnerType ownerType)
    {
        switch (ownerType)
        {
        case Simulation::SpawnerOwnerType_ONE:   return 1;
        case Simulation::SpawnerOwnerType_TWO:   return 2;
        case Simulation::SpawnerOwnerType_THREE: return 3;
        case Simulation::SpawnerOwnerType_FOUR:  return 4;
        case Simulation::SpawnerOwnerType_SEQUENTIAL:
        default:
            // Deterministic controller for sequential ownership mode
            return 1;
        }
    }

    static int ResolveSpawnedOwnerId(const Simulation::SpawnerOwnerType ownerType, uint32_t spawnCounter)
    {
        // Returns ECS/network owner slot in 0..3.
        // Peer ids in config are 1..4 and are compared as (peerId - 1).
        if (ownerType == Simulation::SpawnerOwnerType_SEQUENTIAL)
            return static_cast<int>(spawnCounter % 4u); // ONE->TWO->THREE->FOUR

        switch (ownerType)
        {
        case Simulation::SpawnerOwnerType_ONE:   return 0;
        case Simulation::SpawnerOwnerType_TWO:   return 1;
        case Simulation::SpawnerOwnerType_THREE: return 2;
        case Simulation::SpawnerOwnerType_FOUR:  return 3;
        default:                                 return 0;
        }
    }

    static glm::quat QuatFromNet(const Net::NetQuat& q)
    {
        return glm::normalize(glm::quat(q.w, q.x, q.y, q.z));
    }

    static Net::NetQuat ToNetQuat(const glm::quat& q)
    {
        Net::NetQuat out{};
        out.x = q.x; out.y = q.y; out.z = q.z; out.w = q.w;
        return out;
    }

    static std::string MaterialFromPayload(const Net::SpawnObjectPayload& p)
    {
        size_t n = 0;
        while (n < sizeof(p.material) && p.material[n] != '\0')
            ++n;
        return std::string(p.material, p.material + n);
    }

} // anonymous namespace

// =========================================================================
// Scenario_FlatbufferScene
// =========================================================================

Scenario_FlatbufferScene::Scenario_FlatbufferScene(const std::string& binPath)
    : m_path(binPath)
{
    // Derive a display name from the file stem as placeholder
    // (overridden by scene->name() once loaded).
    const auto lastSlash = m_path.find_last_of("/\\");
    const auto stem      = (lastSlash == std::string::npos)
                               ? m_path
                               : m_path.substr(lastSlash + 1);
    const auto dotPos    = stem.rfind('.');
    m_displayName        = (dotPos == std::string::npos) ? stem : stem.substr(0, dotPos);
}

Scenario_FlatbufferScene::~Scenario_FlatbufferScene() = default;

const char* Scenario_FlatbufferScene::Name() const
{
    return m_displayName.c_str();
}

void Scenario_FlatbufferScene::OnUnload(World& /*world*/)
{
    // world.Clear() is called by ScenarioManager; nothing extra needed here.
    m_spawnerRuntime.clear();
    m_pendingSpawnEvents.clear();
    m_spawnerElapsedSec = 0.0f;
}

void Scenario_FlatbufferScene::OnLoad(World& world)
{
    // ----- Load the .bin file -----
    if (!m_loader)
        m_loader = std::make_unique<SimIO::SceneLoader>();

    if (!m_loader->Load(m_path))
    {
        std::cerr << "[SceneFlatbuffer] Failed to load \"" << m_path
                  << "\": " << m_loader->LastError() << "\n";
        return;
    }

    const Simulation::Scene* scene = m_loader->GetScene();

    // ----- Scene-level metadata -----
    m_gravityOn = scene->gravity_on();

    if (scene->name())
        m_displayName = scene->name()->str();

    // ----- Build material lookup tables -----
    const auto densityMap = BuildDensityMap(scene);

    // ----- Scene dump header -----
    std::cout << "[SceneFlatbuffer] Scene \"" << m_displayName
              << "\" gravity_on=" << (m_gravityOn ? "true" : "false") << "\n";

    // ----- Add a directional light (scene-global) -----
    {
        Entity lightE = world.createEntity();
        DirectionalLightComponent sun{};
        sun.direction = glm::normalize(glm::vec3(-0.3f, -1.0f, -0.2f));
        sun.color     = glm::vec3(1.0f, 1.0f, 1.0f);
        sun.intensity = 2.0f;
        world.addComponent(lightE, sun);
    }

    const unsigned objectCount = scene->objects() ? scene->objects()->size() : 0;

    // ----- Spawn entities in deterministic order -----
    //
    // world.Clear() has already been called by ScenarioManager before OnLoad.
    // nextEntity starts at 0, so the first createEntity() yields 0 which
    // matches object index 0, giving identical IDs across all peers.
    // 
    //

    for (unsigned i = 0; i < objectCount; ++i)
    {
        const Simulation::Object* obj = scene->objects()->Get(i);
        if (!obj) continue;

        // ----- Resolved object name -----
        std::string objName = SimIO::ToStdStringOrEmpty(obj->name());
        if (objName.empty())
            objName = "object " + std::to_string(i);
        // ----- Material -----
        const std::string matName = obj->material()
                                        ? obj->material()->str()
                                        : std::string{};
        const float density = LookupDensity(densityMap, matName);
        const ResolvedMaterial matProps =
            LookupMaterialInteraction(scene, matName);

        // ----- Transform -----
        const Simulation::Transform* fbTransform = obj->transform();
        const glm::vec3 pos       = SimIO::ToPosition(fbTransform);
        const glm::quat ori       = SimIO::ToOrientationQuat(fbTransform);
        const float uniformScale  = SimIO::ToUniformScale_MaxAbs(fbTransform, 1.0f);

        // ----- Shape -----
        const ShapeData rawShape  = SimIO::ToShapeData(obj);


        // ----- Behaviour / collision type -----
        const BehaviourType behaviour =
            SimIO::ToBehaviourType(obj->behaviour_type());
        const CollisionType  colType  =
            SimIO::ToCollisionType(obj->collision_type());

        // ----- Scene dump per-object -----
        std::cout << "  [" << i << "] name=\"" << objName
                  << "\" behaviour=" << BehaviourStr(obj->behaviour_type())
                  << " collision=" << CollisionTypeStr(obj->collision_type())
                  << " material=\"" << matName << "\""
                  << " shape=" << ShapeDumpStr(rawShape, uniformScale) << "\n";

        // ----- Create entity (ID = i for the physics objects block) -----
        Entity e = world.createEntity();

        // ----- NameComponent -----
        world.addComponent(e, NameComponent{ objName });

        // ----- OwnerComponent -----
        {
            OwnerComponent oc{};
            oc.ownerId = -1; // Static/Animated = owned by all

            if (obj->behaviour_type() == Simulation::Behaviour_SimulatedObject)
            {
                const auto* sim = obj->behaviour_as_SimulatedObject();
                if (sim)
                {
                    // ObjectOwnerType is ONE..FOUR -> convert to 0..3
                    oc.ownerId = (int)sim->owner() - 1;
                }
                else
                {
                    oc.ownerId = 0; // fallback
                }
            }

            world.addComponent(e, oc);
        }

        // ----- TransformComponent -----
        {
            TransformComponent tr{};
            tr.position = pos;
            // Store orientation as Euler (yaw, pitch, roll) in radians
            // derived from the quaternion for the TransformComponent.
            // The physics body uses the quaternion directly.
            const glm::vec3 euler = glm::eulerAngles(ori); // pitch, yaw, roll
            tr.rotation = euler;
            tr.scale    = glm::vec3(uniformScale);
            world.addComponent(e, tr);
        }

        // ----- ShapeComponent -----
        {
            ShapeComponent sc{};
            // FlatBuffer policy: keep authored (raw) shape dimensions here.
            // Collision scaling is applied once via TransformComponent.scale.
            sc.shape         = rawShape;
            sc.collisionType = colType;
            world.addComponent(e, sc);
        }

        // ----- RenderMeshComponent -----
        {
            RenderMeshComponent rmc{};
            rmc.meshName    = MeshNameForShape(rawShape);
            rmc.textureName = "sky3.png";
            world.addComponent(e, rmc);
        }

        // ----- MaterialComponent -----
        {
            MaterialComponent mat{};
            mat.shadingModel  = ShadingModel::Phong;
            mat.diffuseColor  = glm::vec3(0.85f, 0.85f, 0.85f);
            mat.specularColor = glm::vec3(1.0f);
            mat.shininess     = 32.0f;
            world.addComponent(e, mat);
        }

        // ----- PhysicsComponent -----
        {
            PhysicsComponent phys{};
            phys.restitution   = matProps.restitution;
            phys.staticFriction  = matProps.staticFric;
            phys.dynamicFriction = matProps.dynamicFric;

            SetupRigidBodyFromFlatbufferObject(
                phys.body, obj,
                [&](const std::string& name) -> float
                {
                    return LookupDensity(densityMap, name);
                });

            world.addComponent(e, phys);
        }
    }

    std::cout << "  Spawned " << objectCount << " object(s).\n";

    m_spawnerRuntime.clear();
    m_pendingSpawnEvents.clear();
    m_spawnerElapsedSec = 0.0f;

    if (scene->spawners() && scene->spawners_type())
    {
        m_spawnerRuntime.resize(scene->spawners()->size());
    }
}

void Scenario_FlatbufferScene::SetLocalPeerId(int peerId)
{
    m_localPeerId = std::max(1, std::min(peerId, 4));
}

bool Scenario_FlatbufferScene::PopPendingSpawn(Net::SpawnObjectPayload& outPayload)
{
    if (m_pendingSpawnEvents.empty())
        return false;
    outPayload = m_pendingSpawnEvents.front();
    m_pendingSpawnEvents.erase(m_pendingSpawnEvents.begin());
    return true;
}

bool Scenario_FlatbufferScene::SpawnFromNetworkEvent(World& world, const Net::SpawnObjectPayload& payload)
{
    Entity e = (Entity)payload.objectId;
    if (world.isAlive(e))
    {
        if (world.hasComponent<PhysicsComponent>(e) || world.hasComponent<NameComponent>(e))
            return true; // duplicate reliable delivery
    }
    else
    {
        e = world.createEntity();
        if ((uint32_t)e != payload.objectId)
        {
            std::cout << "[Spawner] Deterministic id mismatch: expected=" << payload.objectId
                << " got=" << (uint32_t)e << "\n";
        }
    }

    const std::string materialName = MaterialFromPayload(payload);
    const Simulation::Scene* scene = (m_loader ? m_loader->GetScene() : nullptr);
    const auto densityMap = BuildDensityMap(scene);
    const float density = LookupDensity(densityMap, materialName);
    const ResolvedMaterial matProps = LookupMaterialInteraction(scene, materialName);

    world.addComponent(e, NameComponent{ "spawned " + std::to_string(payload.objectId) });

    OwnerComponent owner{};
    owner.ownerId = std::max(0, std::min((int)payload.ownerId, 3));
    world.addComponent(e, owner);

    TransformComponent tr{};
    tr.position = glm::vec3(payload.pos.x, payload.pos.y, payload.pos.z);
    const glm::quat q = QuatFromNet(payload.rot);
    tr.rotation = glm::eulerAngles(q);
    tr.scale = glm::vec3(1.0f);
    world.addComponent(e, tr);

    ShapeComponent sc{};
    switch ((Net::SpawnShapeType)payload.shapeType)
    {
    case Net::SpawnShapeType::Sphere:
        sc.shape = SphereShape{ std::max(0.01f, payload.radius) };
        break;
    case Net::SpawnShapeType::Cylinder:
        sc.shape = CylinderShape{ std::max(0.01f, payload.radius), std::max(0.01f, payload.height) };
        break;
    case Net::SpawnShapeType::Capsule:
        sc.shape = CapsuleShape{ std::max(0.01f, payload.radius), std::max(0.01f, payload.height) };
        break;
    case Net::SpawnShapeType::Cuboid:
    default:
        sc.shape = CuboidShape{ glm::vec3(
            std::max(0.01f, payload.sizeX),
            std::max(0.01f, payload.sizeY),
            std::max(0.01f, payload.sizeZ)) };
        break;
    }
    sc.collisionType = CollisionType::SOLID;
    world.addComponent(e, sc);

    RenderMeshComponent rmc{};
    rmc.meshName = MeshNameForShape(sc.shape);
    rmc.textureName = "sky3.png";
    world.addComponent(e, rmc);

    MaterialComponent mat{};
    mat.shadingModel = ShadingModel::Phong;
    mat.diffuseColor = glm::vec3(0.85f, 0.85f, 0.85f);
    mat.specularColor = glm::vec3(1.0f);
    mat.shininess = 32.0f;
    world.addComponent(e, mat);

    PhysicsComponent phys{};
    phys.density = density;
    phys.restitution = matProps.restitution;
    phys.staticFriction = matProps.staticFric;
    phys.dynamicFriction = matProps.dynamicFric;
    world.addComponent(e, phys);

    VelocityComponent vel{};
    vel.linearVelocity = glm::vec3(payload.linVel.x, payload.linVel.y, payload.linVel.z);
    vel.angularVelocity = glm::vec3(payload.angVel.x, payload.angVel.y, payload.angVel.z);
    world.addComponent(e, vel);

    return true;
}

void Scenario_FlatbufferScene::Update(World& world, float dt)
{
    if (!m_loader || !m_loader->IsLoaded())
        return;

    const Simulation::Scene* scene = m_loader->GetScene();
    if (!scene || !scene->spawners() || !scene->spawners_type())
        return;

    m_spawnerElapsedSec += std::max(0.0f, dt);

    const auto* spawners = scene->spawners();
    const auto* types = scene->spawners_type();
    if (m_spawnerRuntime.size() != spawners->size())
        m_spawnerRuntime.resize(spawners->size());

    for (uint32_t i = 0; i < spawners->size(); ++i)
    {
        const Simulation::SpawnerType spType = (Simulation::SpawnerType)types->Get(i);
        const void* spPtr = spawners->Get(i);
        if (!spPtr) continue;

        const Simulation::BaseSpawner* base = nullptr;
        const Simulation::FloatRange* radiusRange = nullptr;
        const Simulation::FloatRange* heightRange = nullptr;
        const Simulation::Vec3Range* sizeRange = nullptr;
        Net::SpawnShapeType netShape = Net::SpawnShapeType::Sphere;

        switch (spType)
        {
        case Simulation::SpawnerType_SphereSpawner:
        {
            const auto* s = static_cast<const Simulation::SphereSpawner*>(spPtr);
            if (!s) continue;
            base = s->base();
            radiusRange = s->radius_range();
            netShape = Net::SpawnShapeType::Sphere;
            break;
        }
        case Simulation::SpawnerType_CylinderSpawner:
        {
            const auto* s = static_cast<const Simulation::CylinderSpawner*>(spPtr);
            if (!s) continue;
            base = s->base();
            radiusRange = s->radius_range();
            heightRange = s->height_range();
            netShape = Net::SpawnShapeType::Cylinder;
            break;
        }
        case Simulation::SpawnerType_CapsuleSpawner:
        {
            const auto* s = static_cast<const Simulation::CapsuleSpawner*>(spPtr);
            if (!s) continue;
            base = s->base();
            radiusRange = s->radius_range();
            heightRange = s->height_range();
            netShape = Net::SpawnShapeType::Capsule;
            break;
        }
        case Simulation::SpawnerType_CuboidSpawner:
        {
            const auto* s = static_cast<const Simulation::CuboidSpawner*>(spPtr);
            if (!s) continue;
            base = s->base();
            sizeRange = s->size_range();
            netShape = Net::SpawnShapeType::Cuboid;
            break;
        }
        default:
            continue;
        }

        if (!base)
            continue;

        if (ResolveControllerPeerId(base->owner()) != m_localPeerId)
            continue;

        auto& rt = m_spawnerRuntime[i];
        const float startTime = std::max(0.0f, base->start_time());
        if (m_spawnerElapsedSec < startTime)
            continue;

        const auto* burst = base->spawn_type_as_SingleBurstSpawn();
        const auto* repeating = base->spawn_type_as_RepeatingSpawn();
        if (!burst && !repeating)
            continue;

        int spawnCountThisTick = 0;
        if (burst)
        {
            if (rt.burstDone) continue;
            spawnCountThisTick = (int)burst->count();
            rt.burstDone = true;
        }
        else if (repeating)
        {
            const uint32_t maxCount = repeating->max_count();
            const float interval = std::max(0.001f, repeating->interval());
            if (rt.emittedCount >= maxCount)
                continue;

            if (rt.nextSpawnTimeSec <= 0.0f)
                rt.nextSpawnTimeSec = startTime;

            while (rt.emittedCount < maxCount && m_spawnerElapsedSec >= rt.nextSpawnTimeSec)
            {
                ++spawnCountThisTick;
                rt.nextSpawnTimeSec += interval;
            }
        }

        for (int n = 0; n < spawnCountThisTick; ++n)
        {
            Net::SpawnObjectPayload payload{};

            Entity e = world.createEntity();
            payload.objectId = (uint32_t)e;

            const uint32_t seed = Hash32((i + 1u) * 0x1f123bb5u + rt.spawnCounter * 2654435761u + payload.objectId);

            glm::vec3 spawnPos{ 0.0f };
            glm::quat spawnRot{ 1, 0, 0, 0 };
            if (const auto* fixed = base->location_as_FixedLocation())
            {
                spawnPos = SimIO::ToPosition(fixed->transform());
                spawnRot = SimIO::ToOrientationQuat(fixed->transform());
            }
            else if (const auto* box = base->location_as_RandomBox())
            {
                const glm::vec3 mn = box->min() ? SimIO::ToGlmVec3(*box->min()) : glm::vec3(0.0f);
                const glm::vec3 mx = box->max() ? SimIO::ToGlmVec3(*box->max()) : glm::vec3(0.0f);
                spawnPos = glm::vec3(
                    std::min(mn.x, mx.x) + (std::max(mn.x, mx.x) - std::min(mn.x, mx.x)) * HashUnitFloat(seed, 11),
                    std::min(mn.y, mx.y) + (std::max(mn.y, mx.y) - std::min(mn.y, mx.y)) * HashUnitFloat(seed, 12),
                    std::min(mn.z, mx.z) + (std::max(mn.z, mx.z) - std::min(mn.z, mx.z)) * HashUnitFloat(seed, 13));
            }
            else if (const auto* sphere = base->location_as_RandomSphere())
            {
                const glm::vec3 c = sphere->center() ? SimIO::ToGlmVec3(*sphere->center()) : glm::vec3(0.0f);
                const float r = std::max(0.0f, sphere->radius());
                const float u = HashUnitFloat(seed, 21);
                const float v = HashUnitFloat(seed, 22);
                const float w = HashUnitFloat(seed, 23);
                const float theta = 2.0f * 3.14159265f * u;
                const float phi = std::acos(2.0f * v - 1.0f);
                const float rad = r * std::cbrt(std::max(0.0f, w));
                const glm::vec3 dir{
                    std::sin(phi) * std::cos(theta),
                    std::sin(phi) * std::sin(theta),
                    std::cos(phi)
                };
                spawnPos = c + dir * rad;
            }

            const glm::vec3 linVel = SampleVec3Range(base->linear_velocity(), glm::vec3(0.0f), seed, 31);
            const glm::vec3 angVelDeg = SampleVec3Range(base->angular_velocity(), glm::vec3(0.0f), seed, 41);
            const glm::vec3 angVel = glm::vec3(
                SimIO::DegToRad(angVelDeg.x),
                SimIO::DegToRad(angVelDeg.y),
                SimIO::DegToRad(angVelDeg.z));

            payload.ownerId = (uint8_t)ResolveSpawnedOwnerId(base->owner(), rt.spawnCounter);
            payload.shapeType = (uint8_t)netShape;
            payload.pos = { spawnPos.x, spawnPos.y, spawnPos.z };
            payload.rot = ToNetQuat(spawnRot);
            payload.linVel = { linVel.x, linVel.y, linVel.z };
            payload.angVel = { angVel.x, angVel.y, angVel.z };

            payload.radius = SampleFloatRange(radiusRange, 0.5f, seed, 51);
            payload.height = SampleFloatRange(heightRange, 1.0f, seed, 52);
            const glm::vec3 sampledSize = SampleVec3Range(sizeRange, glm::vec3(1.0f), seed, 53);
            payload.sizeX = sampledSize.x;
            payload.sizeY = sampledSize.y;
            payload.sizeZ = sampledSize.z;

            const std::string material = SimIO::ToStdStringOrEmpty(base->material());
            if (!material.empty())
            {
                std::strncpy(payload.material, material.c_str(), sizeof(payload.material) - 1);
            }

            SpawnFromNetworkEvent(world, payload);
            m_pendingSpawnEvents.push_back(payload);

            ++rt.spawnCounter;
            ++rt.emittedCount;
        }
    }
}
