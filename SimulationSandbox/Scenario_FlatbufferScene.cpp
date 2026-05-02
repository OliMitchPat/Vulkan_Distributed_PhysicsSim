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
            if constexpr (std::is_same_v<T, CylinderShape>) return "sphere.obj";
            if constexpr (std::is_same_v<T, CapsuleShape>)  return "sphere.obj";
            if constexpr (std::is_same_v<T, PlaneShape>)    return "Plane.obj";
            return "sphere.obj";
        }, shape);
    }

    // Scale the shape dimensions by the uniform scale derived from the transform.
    ShapeData ScaleShape(const ShapeData& shape, float u)
    {
        return std::visit([u](auto&& s) -> ShapeData
        {
            using T = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<T, SphereShape>)
                return SphereShape{ s.radius * u };
            if constexpr (std::is_same_v<T, CuboidShape>)
                return CuboidShape{ s.size * u };
            if constexpr (std::is_same_v<T, CylinderShape>)
                return CylinderShape{ s.radius * u, s.height * u };
            if constexpr (std::is_same_v<T, CapsuleShape>)
                return CapsuleShape{ s.radius * u, s.height * u };
            if constexpr (std::is_same_v<T, PlaneShape>)
                return s;  // plane: scale doesn't change the normal
            return s;
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

    if (!scene->objects() || scene->objects()->size() == 0)
    {
        std::cout << "  (no objects in scene)\n";
        return;
    }

    // ----- Spawn entities in deterministic order -----
    //
    // world.Clear() has already been called by ScenarioManager before OnLoad.
    // nextEntity starts at 0, so the first createEntity() yields 0 which
    // matches object index 0, giving identical IDs across all peers.
    //
    const unsigned objectCount = scene->objects()->size();

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
        const ShapeData shape     = ScaleShape(rawShape, uniformScale);

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
            sc.shape         = shape;
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
}
