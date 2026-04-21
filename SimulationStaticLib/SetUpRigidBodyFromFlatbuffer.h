#pragma once

#include "RigidBody.h"
#include "BodyDispatcher.h"
#include "FlatbufferConvert.h"

#include <algorithm>
#include <string>

template <typename DensityLookupFn>
inline void SetupRigidBodyFromFlatbufferObject(
    RigidBody& body,
    const Simulation::Object* obj,
    DensityLookupFn&& getDensityByMaterialName)
{
    if (!obj) return;

    // Pose
    const auto* t = obj->transform();
    body.SetPosition(SimIO::ToPosition(t));
    body.SetOrientation(SimIO::ToOrientationQuat(t));

    const BehaviourType behaviour = SimIO::ToBehaviourType(obj->behaviour_type());
    const CollisionType collisionType = SimIO::ToCollisionType(obj->collision_type());

    // Material density (default 0 => static by your BodySetup policy)
    float density = 0.0f;
    if (obj->material())
        density = std::max(0.0f, getDensityByMaterialName(obj->material()->str()));

    // Shape (with defaults)
    const ShapeData shape = SimIO::ToShapeData(obj);

    // Initial velocities for simulated objects
    if (behaviour == BehaviourType::Simulated)
    {
        const auto* sim = obj->behaviour_as_SimulatedObject();
        const auto* state = (sim) ? sim->initial_state() : nullptr;

        body.SetLinearVelocity(SimIO::ToLinearVelocity(state));
        body.SetAngularVelocity(SimIO::ToAngularVelocityRad(state));
    }
    else
    {
        body.SetLinearVelocity(glm::vec3(0.0f));
        body.SetAngularVelocity(glm::vec3(0.0f));
    }

    SetupBodyFromShape(body, shape, density, behaviour, collisionType);
}