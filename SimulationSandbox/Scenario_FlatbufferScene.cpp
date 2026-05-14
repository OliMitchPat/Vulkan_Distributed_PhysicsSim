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
#include <cmath>

// ---- Fallback material constants ----------------------------------------
namespace
{
    constexpr float kFallbackDensity = 1000.0f;
    constexpr float kFallbackRestitution = 0.5f;
    constexpr float kFallbackStaticFric = 0.5f;
    constexpr float kFallbackDynamicFric = 0.3f;

    struct ResolvedMaterial
    {
        float density = kFallbackDensity;
        float restitution = kFallbackRestitution;
        float staticFric = kFallbackStaticFric;
        float dynamicFric = kFallbackDynamicFric;
    };

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
                result.staticFric = inter->static_friction();
                result.dynamicFric = inter->dynamic_friction();
                return result;
            }
        }

        return result;
    }

    static glm::vec3 ColourFromMaterialName(const std::string& name)
    {
        if (name.empty())
            return glm::vec3(0.85f, 0.85f, 0.85f);

        // FNV-1a hash so every peer generates the same colour for the same material.
        uint32_t h = 2166136261u;
        for (unsigned char c : name)
        {
            h ^= c;
            h *= 16777619u;
        }

        // Bright enough to show under lighting.
        const float r = 0.10f + 0.90f * static_cast<float>((h >> 0) & 0xFF) / 255.0f;
        const float g = 0.10f + 0.90f * static_cast<float>((h >> 8) & 0xFF) / 255.0f;
        const float b = 0.10f + 0.90f * static_cast<float>((h >> 16) & 0xFF) / 255.0f;

        return glm::vec3(r, g, b);
    }

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

    static float SampleFloatRange(
        const Simulation::FloatRange* range,
        float fallback,
        uint32_t seed,
        uint32_t salt)
    {
        if (!range) return fallback;

        const float a = range->min();
        const float b = range->max();
        const float lo = std::min(a, b);
        const float hi = std::max(a, b);

        if (std::abs(hi - lo) <= 1e-6f)
            return lo;

        return lo + (hi - lo) * HashUnitFloat(seed, salt);
    }

    static glm::vec3 SampleVec3Range(
        const Simulation::Vec3Range* range,
        const glm::vec3& fallback,
        uint32_t seed,
        uint32_t saltBase)
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
            return 1;
        }
    }

    static int ResolveSpawnedOwnerId(
        const Simulation::SpawnerOwnerType ownerType,
        uint32_t spawnCounter)
    {
        if (ownerType == Simulation::SpawnerOwnerType_SEQUENTIAL)
            return static_cast<int>(spawnCounter % 4u);

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
        out.x = q.x;
        out.y = q.y;
        out.z = q.z;
        out.w = q.w;
        return out;
    }

    static std::string MaterialFromPayload(const Net::SpawnObjectPayload& p)
    {
        size_t n = 0;
        while (n < sizeof(p.material) && p.material[n] != '\0')
            ++n;

        return std::string(p.material, p.material + n);
    }

    // ---------------------------------------------------------------------
    // Animated object helpers
    // ---------------------------------------------------------------------

    static float ApplyEasing(float t, AnimatedEasing easing)
    {
        t = std::clamp(t, 0.0f, 1.0f);

        if (easing == AnimatedEasing::Smoothstep)
            return t * t * (3.0f - 2.0f * t);

        return t;
    }

    static float LastWaypointTime(const AnimatedPathComponent& path)
    {
        if (path.waypoints.empty())
            return 0.0f;

        return path.waypoints.back().time;
    }

    static float ResolveAnimatedLocalTime(const AnimatedPathComponent& path)
    {
        const float endTime = LastWaypointTime(path);

        if (endTime <= 0.0f)
            return 0.0f;

        switch (path.mode)
        {
        case AnimatedPathMode::Stop:
            return std::min(path.elapsed, endTime);

        case AnimatedPathMode::Loop:
        {
            const float duration = std::max(path.totalDuration, endTime);

            if (duration <= 0.0f)
                return 0.0f;

            return std::fmod(path.elapsed, duration);
        }

        case AnimatedPathMode::Reverse:
        {
            const float cycle = endTime * 2.0f;

            if (cycle <= 0.0f)
                return 0.0f;

            const float t = std::fmod(path.elapsed, cycle);

            if (t <= endTime)
                return t;

            return cycle - t;
        }

        default:
            return 0.0f;
        }
    }

    static void EvaluateAnimatedPath(
        const AnimatedPathComponent& path,
        glm::vec3& outPosition,
        glm::quat& outRotation)
    {
        if (path.waypoints.empty())
        {
            outPosition = glm::vec3(0.0f);
            outRotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            return;
        }

        if (path.waypoints.size() == 1)
        {
            outPosition = path.waypoints.front().position;
            outRotation = path.waypoints.front().rotation;
            return;
        }

        const float localTime = ResolveAnimatedLocalTime(path);

        // LOOP mode has one extra segment from the last waypoint back to the first.
        if (path.mode == AnimatedPathMode::Loop)
        {
            const float lastTime = LastWaypointTime(path);
            const float duration = std::max(path.totalDuration, lastTime);

            if (localTime >= lastTime && duration > lastTime)
            {
                const auto& a = path.waypoints.back();
                const auto& b = path.waypoints.front();

                const float rawT = (localTime - lastTime) / (duration - lastTime);
                const float t = ApplyEasing(rawT, path.easing);

                outPosition = glm::mix(a.position, b.position, t);
                outRotation = glm::slerp(a.rotation, b.rotation, t);
                return;
            }
        }

        for (std::size_t i = 0; i + 1 < path.waypoints.size(); ++i)
        {
            const auto& a = path.waypoints[i];
            const auto& b = path.waypoints[i + 1];

            if (localTime >= a.time && localTime <= b.time)
            {
                const float segmentDuration = std::max(0.0001f, b.time - a.time);
                const float rawT = (localTime - a.time) / segmentDuration;
                const float t = ApplyEasing(rawT, path.easing);

                outPosition = glm::mix(a.position, b.position, t);
                outRotation = glm::slerp(a.rotation, b.rotation, t);
                return;
            }
        }

        outPosition = path.waypoints.back().position;
        outRotation = path.waypoints.back().rotation;
    }

} // anonymous namespace

// =========================================================================
// Scenario_FlatbufferScene
// =========================================================================

Scenario_FlatbufferScene::Scenario_FlatbufferScene(const std::string& binPath)
    : m_path(binPath)
{
    const auto lastSlash = m_path.find_last_of("/\\");
    const auto stem = (lastSlash == std::string::npos)
        ? m_path
        : m_path.substr(lastSlash + 1);

    const auto dotPos = stem.rfind('.');
    m_displayName = (dotPos == std::string::npos)
        ? stem
        : stem.substr(0, dotPos);
}

Scenario_FlatbufferScene::~Scenario_FlatbufferScene() = default;

const char* Scenario_FlatbufferScene::Name() const
{
    return m_displayName.c_str();
}

void Scenario_FlatbufferScene::OnUnload(World& /*world*/)
{
    m_spawnerRuntime.clear();
    m_pendingSpawnEvents.clear();
    m_spawnerElapsedSec = 0.0f;
}

void Scenario_FlatbufferScene::OnLoad(World& world)
{
    if (!m_loader)
        m_loader = std::make_unique<SimIO::SceneLoader>();

    if (!m_loader->Load(m_path))
    {
        std::cerr << "[SceneFlatbuffer] Failed to load \"" << m_path
            << "\": " << m_loader->LastError() << "\n";
        return;
    }

    const Simulation::Scene* scene = m_loader->GetScene();

    m_gravityOn = scene->gravity_on();

    if (scene->name())
        m_displayName = scene->name()->str();

    const auto densityMap = BuildDensityMap(scene);

    std::cout << "[SceneFlatbuffer] Scene \"" << m_displayName
        << "\" gravity_on=" << (m_gravityOn ? "true" : "false") << "\n";

    {
        Entity lightE = world.createEntity();

        DirectionalLightComponent sun{};
        sun.direction = glm::normalize(glm::vec3(-0.3f, -1.0f, -0.2f));
        sun.color = glm::vec3(1.0f, 1.0f, 1.0f);
        sun.intensity = 2.0f;

        world.addComponent(lightE, sun);
    }

    const unsigned objectCount = scene->objects() ? scene->objects()->size() : 0;

    // ------------------------------------------------------------
    // Scene-loaded ownership:
    // Use exactly what the FlatBuffer says.
    //
    // We previously had a round-robin fallback here, but that caused
    // tightly-coupled scenes like Newton's Cradle to break because
    // chained collision impulses were split across networked owners.
    //
    // Distributed ownership should be demonstrated via correctly
    // authored FlatBuffer ownership and/or spawner ownership.
    // ------------------------------------------------------------
    const bool shouldRoundRobinSceneOwnership = false;

    unsigned simulatedObjectIndexForOwnership = 0;

    std::cout << "[OwnerDebug] Scene ownership fallback: "
        << (shouldRoundRobinSceneOwnership ? "ENABLED" : "DISABLED")
        << "\n";

    for (unsigned i = 0; i < objectCount; ++i)
    {
        const Simulation::Object* obj = scene->objects()->Get(i);
        if (!obj) continue;

        std::string objName = SimIO::ToStdStringOrEmpty(obj->name());
        if (objName.empty())
            objName = "object " + std::to_string(i);

        // ------------------------------------------------------------
        // Debug: show owner value read directly from the FlatBuffer.
        // - Simulated objects should have owner 0..3.
        // - Static / animated objects are effectively local/all-peer,
        //   so we leave them as -1 for this debug output.
        // ------------------------------------------------------------
        int fbOwnerDebug = -1;

        if (obj->behaviour_type() == Simulation::Behaviour_SimulatedObject)
        {
            const auto* sim = obj->behaviour_as_SimulatedObject();

            if (sim)
            {
                fbOwnerDebug = std::clamp(static_cast<int>(sim->owner()), 0, 3);
            }
        }

        std::cout << "[FlatbufferOwnerDebug] object=\"" << objName
            << "\" behaviour=" << BehaviourStr(obj->behaviour_type())
            << " fbOwner=" << fbOwnerDebug;

        if (fbOwnerDebug >= 0)
            std::cout << " peer=" << (fbOwnerDebug + 1);
        else
            std::cout << " peer=ALL_OR_STATIC";

        std::cout << "\n";

        const std::string matName = obj->material()
            ? obj->material()->str()
            : std::string{};

        const float density = LookupDensity(densityMap, matName);
        const ResolvedMaterial matProps = LookupMaterialInteraction(scene, matName);

        const Simulation::Transform* fbTransform = obj->transform();
        const glm::vec3 pos = SimIO::ToPosition(fbTransform);
        const glm::quat ori = SimIO::ToOrientationQuat(fbTransform);
        const float uniformScale = SimIO::ToUniformScale_MaxAbs(fbTransform, 1.0f);

        const ShapeData rawShape = SimIO::ToShapeData(obj);

        const BehaviourType behaviour =
            SimIO::ToBehaviourType(obj->behaviour_type());

        const CollisionType colType =
            SimIO::ToCollisionType(obj->collision_type());

        std::cout << "  [" << i << "] name=\"" << objName
            << "\" behaviour=" << BehaviourStr(obj->behaviour_type())
            << " collision=" << CollisionTypeStr(obj->collision_type())
            << " material=\"" << matName << "\""
            << " shape=" << ShapeDumpStr(rawShape, uniformScale)
            << "\n";

        Entity e = world.createEntity();

        world.addComponent(e, NameComponent{ objName });

        {
            OwnerComponent oc{};
            oc.ownerId = -1;

            if (obj->behaviour_type() == Simulation::Behaviour_SimulatedObject)
            {
                const auto* sim = obj->behaviour_as_SimulatedObject();

                const int fbOwner = sim
                    ? std::clamp(static_cast<int>(sim->owner()), 0, 3)
                    : 0;

                if (shouldRoundRobinSceneOwnership)
                {
                    // Disabled for now because shouldRoundRobinSceneOwnership is false.
                    // Kept here so we can re-enable a scene-specific fallback later if needed.
                    oc.ownerId = simulatedObjectIndexForOwnership % 4;
                }
                else
                {
                    oc.ownerId = fbOwner;
                }

                ++simulatedObjectIndexForOwnership;
            }

            world.addComponent(e, oc);
        }

        {
            TransformComponent tr{};
            tr.position = pos;
            tr.rotation = glm::eulerAngles(ori);
            tr.scale = glm::vec3(uniformScale);
            world.addComponent(e, tr);
        }

        {
            ShapeComponent sc{};
            sc.shape = rawShape;
            sc.collisionType = colType;
            world.addComponent(e, sc);
        }

        {
            RenderMeshComponent rmc{};
            rmc.meshName = MeshNameForShape(rawShape);
            rmc.textureName = "white.png";
            world.addComponent(e, rmc);
        }

        {
            MaterialComponent mat{};
            mat.shadingModel = ShadingModel::Phong;
            mat.diffuseColor = ColourFromMaterialName(matName);
            mat.specularColor = glm::vec3(1.0f);
            mat.shininess = 32.0f;
            mat.alpha = 1.0f;

            if (colType == CollisionType::CONTAINER)
            {
                mat.diffuseColor = glm::vec3(0.35f, 0.65f, 1.0f);
                mat.specularColor = glm::vec3(0.2f);
                mat.shininess = 8.0f;
                mat.alpha = 0.18f;

                mat.castsShadows = false;
                mat.receivesShadows = false;
            }

            world.addComponent(e, mat);
        }

        {
            PhysicsComponent phys{};

            phys.density = density;

            phys.restitution = matProps.restitution;
            phys.staticFriction = matProps.staticFric;
            phys.dynamicFriction = matProps.dynamicFric;

            SetupRigidBodyFromFlatbufferObject(
                phys.body,
                obj,
                [&](const std::string& name) -> float
                {
                    return LookupDensity(densityMap, name);
                });

            world.addComponent(e, phys);
        }

        // --------------------------------------------------
        // AnimatedObject loading
        // --------------------------------------------------
        if (obj->behaviour_type() == Simulation::Behaviour_AnimatedObject)
        {
            const auto* anim = obj->behaviour_as_AnimatedObject();

            if (anim && anim->waypoints() && anim->waypoints()->size() >= 2)
            {
                AnimatedPathComponent path{};
                path.totalDuration = anim->total_duration();
                path.elapsed = 0.0f;

                path.easing =
                    anim->easing() == Simulation::EasingType_SMOOTHSTEP
                    ? AnimatedEasing::Smoothstep
                    : AnimatedEasing::Linear;

                switch (anim->path_mode())
                {
                case Simulation::PathMode_LOOP:
                    path.mode = AnimatedPathMode::Loop;
                    break;

                case Simulation::PathMode_REVERSE:
                    path.mode = AnimatedPathMode::Reverse;
                    break;

                case Simulation::PathMode_STOP:
                default:
                    path.mode = AnimatedPathMode::Stop;
                    break;
                }

                for (unsigned w = 0; w < anim->waypoints()->size(); ++w)
                {
                    const auto* fbWp = anim->waypoints()->Get(w);

                    if (!fbWp || !fbWp->position() || !fbWp->rotation())
                        continue;

                    AnimatedWaypoint wp{};
                    wp.position = SimIO::ToGlmVec3(*fbWp->position());
                    wp.rotation = SimIO::ToOrientationQuat(*fbWp->rotation());
                    wp.time = fbWp->time();

                    path.waypoints.push_back(wp);
                }

                std::sort(
                    path.waypoints.begin(),
                    path.waypoints.end(),
                    [](const AnimatedWaypoint& a, const AnimatedWaypoint& b)
                    {
                        return a.time < b.time;
                    });

                if (path.waypoints.size() >= 2)
                {
                    world.addComponent(e, path);

                    if (auto* phys = world.getComponent<PhysicsComponent>(e))
                    {
                        phys->body.SetMotionType(BodyMotionType::Kinematic);
                        phys->body.SetPosition(path.waypoints.front().position);
                        phys->body.SetOrientation(path.waypoints.front().rotation);
                        phys->body.SetLinearVelocity(glm::vec3(0.0f));
                        phys->body.SetAngularVelocity(glm::vec3(0.0f));
                        phys->initialized = true;
                    }

                    if (auto* tr = world.getComponent<TransformComponent>(e))
                    {
                        tr->position = path.waypoints.front().position;
                        tr->rotation = glm::eulerAngles(path.waypoints.front().rotation);
                    }

                    VelocityComponent vel{};
                    vel.linearVelocity = glm::vec3(0.0f);
                    vel.angularVelocity = glm::vec3(0.0f);
                    world.addComponent(e, vel);

                    std::cout << "    animated waypoints=" << path.waypoints.size()
                        << " total_duration=" << path.totalDuration << "\n";
                }
            }
            else
            {
                std::cout << "    animated object has fewer than 2 valid waypoints; skipping animation\n";
            }
        }
    }

    std::cout << "  Spawned " << objectCount << " object(s).\n";

    m_spawnerRuntime.clear();
    m_pendingSpawnEvents.clear();
    m_spawnerElapsedSec = 0.0f;

    if (scene->spawners() && scene->spawners_type())
        m_spawnerRuntime.resize(scene->spawners()->size());
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

bool Scenario_FlatbufferScene::SpawnFromNetworkEvent(
    World& world,
    const Net::SpawnObjectPayload& payload)
{
    Entity e = static_cast<Entity>(payload.objectId);

    if (world.isAlive(e))
    {
        if (world.hasComponent<PhysicsComponent>(e) || world.hasComponent<NameComponent>(e))
            return true;
    }
    else
    {
        e = world.createEntity();

        if (static_cast<uint32_t>(e) != payload.objectId)
        {
            std::cout << "[Spawner] Deterministic id mismatch: expected=" << payload.objectId
                << " got=" << static_cast<uint32_t>(e) << "\n";
        }
    }

    const std::string materialName = MaterialFromPayload(payload);
    const Simulation::Scene* scene = (m_loader ? m_loader->GetScene() : nullptr);

    const auto densityMap = BuildDensityMap(scene);
    const float density = LookupDensity(densityMap, materialName);
    const ResolvedMaterial matProps = LookupMaterialInteraction(scene, materialName);

    world.addComponent(e, NameComponent{ "spawned " + std::to_string(payload.objectId) });

    OwnerComponent owner{};
    owner.ownerId = std::max(0, std::min(static_cast<int>(payload.ownerId), 3));
    world.addComponent(e, owner);

    TransformComponent tr{};
    tr.position = glm::vec3(payload.pos.x, payload.pos.y, payload.pos.z);

    const glm::quat q = QuatFromNet(payload.rot);
    tr.rotation = glm::eulerAngles(q);
    tr.scale = glm::vec3(1.0f);
    world.addComponent(e, tr);

    ShapeComponent sc{};

    switch (static_cast<Net::SpawnShapeType>(payload.shapeType))
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
    rmc.textureName = "white.png";
    world.addComponent(e, rmc);

    MaterialComponent mat{};
    mat.shadingModel = ShadingModel::Phong;
    mat.diffuseColor = ColourFromMaterialName(materialName);
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

    if (!scene)
        return;

    // --------------------------------------------------
    // Animated object update
    // --------------------------------------------------
    // This should run before PhysicsSystem and CollisionSystem in the main loop.
    // The animated object is moved externally and assigned a velocity so
    // simulated objects collide with it as a moving kinematic object.
    world.forEach<AnimatedPathComponent, PhysicsComponent>(
        [&](Entity e, AnimatedPathComponent& path, PhysicsComponent& phys)
        {
            if (path.waypoints.size() < 2)
                return;

            const float safeDt = std::max(dt, 0.0001f);
            const glm::vec3 oldPosition = phys.body.Position();

            path.elapsed += std::max(0.0f, dt);

            glm::vec3 newPosition{ 0.0f };
            glm::quat newRotation{ 1.0f, 0.0f, 0.0f, 0.0f };
            EvaluateAnimatedPath(path, newPosition, newRotation);

            glm::vec3 linearVelocity = (newPosition - oldPosition) / safeDt;

            if (path.mode == AnimatedPathMode::Stop && path.elapsed >= LastWaypointTime(path))
                linearVelocity = glm::vec3(0.0f);

            phys.body.SetMotionType(BodyMotionType::Kinematic);
            phys.body.SetPosition(newPosition);
            phys.body.SetOrientation(newRotation);
            phys.body.SetLinearVelocity(linearVelocity);
            phys.body.SetAngularVelocity(glm::vec3(0.0f));
            phys.initialized = true;

            if (auto* tr = world.getComponent<TransformComponent>(e))
            {
                tr->position = newPosition;
                tr->rotation = glm::eulerAngles(newRotation);
            }

            if (auto* vel = world.getComponent<VelocityComponent>(e))
            {
                vel->linearVelocity = linearVelocity;
                vel->angularVelocity = glm::vec3(0.0f);
            }
        });

    if (!scene->spawners() || !scene->spawners_type())
        return;

    m_spawnerElapsedSec += std::max(0.0f, dt);

    const auto* spawners = scene->spawners();
    const auto* types = scene->spawners_type();

    if (m_spawnerRuntime.size() != spawners->size())
        m_spawnerRuntime.resize(spawners->size());

    for (uint32_t i = 0; i < spawners->size(); ++i)
    {
        const Simulation::SpawnerType spType =
            static_cast<Simulation::SpawnerType>(types->Get(i));

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
            if (rt.burstDone)
                continue;

            spawnCountThisTick = static_cast<int>(burst->count());
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
            payload.objectId = static_cast<uint32_t>(e);

            const uint32_t seed =
                Hash32((i + 1u) * 0x1f123bb5u
                    + rt.spawnCounter * 2654435761u
                    + payload.objectId);

            glm::vec3 spawnPos{ 0.0f };
            glm::quat spawnRot{ 1.0f, 0.0f, 0.0f, 0.0f };

            if (const auto* fixed = base->location_as_FixedLocation())
            {
                spawnPos = SimIO::ToPosition(fixed->transform());
                spawnRot = SimIO::ToOrientationQuat(fixed->transform());
            }
            else if (const auto* box = base->location_as_RandomBox())
            {
                const glm::vec3 mn =
                    box->min() ? SimIO::ToGlmVec3(*box->min()) : glm::vec3(0.0f);

                const glm::vec3 mx =
                    box->max() ? SimIO::ToGlmVec3(*box->max()) : glm::vec3(0.0f);

                spawnPos = glm::vec3(
                    std::min(mn.x, mx.x) + (std::max(mn.x, mx.x) - std::min(mn.x, mx.x)) * HashUnitFloat(seed, 11),
                    std::min(mn.y, mx.y) + (std::max(mn.y, mx.y) - std::min(mn.y, mx.y)) * HashUnitFloat(seed, 12),
                    std::min(mn.z, mx.z) + (std::max(mn.z, mx.z) - std::min(mn.z, mx.z)) * HashUnitFloat(seed, 13));
            }
            else if (const auto* sphere = base->location_as_RandomSphere())
            {
                const glm::vec3 c =
                    sphere->center() ? SimIO::ToGlmVec3(*sphere->center()) : glm::vec3(0.0f);

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

            const glm::vec3 linVel =
                SampleVec3Range(base->linear_velocity(), glm::vec3(0.0f), seed, 31);

            const glm::vec3 angVelDeg =
                SampleVec3Range(base->angular_velocity(), glm::vec3(0.0f), seed, 41);

            const glm::vec3 angVel = glm::vec3(
                SimIO::DegToRad(angVelDeg.x),
                SimIO::DegToRad(angVelDeg.y),
                SimIO::DegToRad(angVelDeg.z));

            payload.ownerId = static_cast<uint8_t>(
                ResolveSpawnedOwnerId(base->owner(), rt.spawnCounter));

            payload.shapeType = static_cast<uint8_t>(netShape);

            payload.pos = { spawnPos.x, spawnPos.y, spawnPos.z };
            payload.rot = ToNetQuat(spawnRot);
            payload.linVel = { linVel.x, linVel.y, linVel.z };
            payload.angVel = { angVel.x, angVel.y, angVel.z };

            payload.radius =
                SampleFloatRange(radiusRange, 0.5f, seed, 51);

            payload.height =
                SampleFloatRange(heightRange, 1.0f, seed, 52);

            const glm::vec3 sampledSize =
                SampleVec3Range(sizeRange, glm::vec3(1.0f), seed, 53);

            payload.sizeX = sampledSize.x;
            payload.sizeY = sampledSize.y;
            payload.sizeZ = sampledSize.z;

            const std::string material = SimIO::ToStdStringOrEmpty(base->material());

            if (!material.empty())
            {
                strncpy_s(
                    payload.material,
                    sizeof(payload.material),
                    material.c_str(),
                    _TRUNCATE
                );
            }

            SpawnFromNetworkEvent(world, payload);
            m_pendingSpawnEvents.push_back(payload);

            ++rt.spawnCounter;
            ++rt.emittedCount;
        }
    }
}
