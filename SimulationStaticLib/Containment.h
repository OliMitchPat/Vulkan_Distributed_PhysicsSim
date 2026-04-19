#pragma once
#include "WorldShapes.h"
#include "CollisionManifold.h"

// Inside convention for Plane container:
// We define "inside" as SignedDistanceToPlane(point) <= 0 (i.e., behind the plane normal).
// If you want the opposite, flip the sign checks in Contain(..., WorldPlane).

// -------- Plane container (half-space) --------
CollisionManifold Contain(const WorldSphere& a, const WorldPlane& container);
CollisionManifold Contain(const WorldCapsule& a, const WorldPlane& container);
CollisionManifold Contain(const WorldCylinder& a, const WorldPlane& container);
CollisionManifold Contain(const WorldOBB& a, const WorldPlane& container);

// -------- Sphere container --------
CollisionManifold Contain(const WorldSphere& a, const WorldSphere& container);
CollisionManifold Contain(const WorldCapsule& a, const WorldSphere& container);
CollisionManifold Contain(const WorldCylinder& a, const WorldSphere& container);
CollisionManifold Contain(const WorldOBB& a, const WorldSphere& container);

// -------- OBB container --------
CollisionManifold Contain(const WorldSphere& a, const WorldOBB& container);
CollisionManifold Contain(const WorldCapsule& a, const WorldOBB& container);
CollisionManifold Contain(const WorldCylinder& a, const WorldOBB& container);
CollisionManifold Contain(const WorldOBB& a, const WorldOBB& container);

// -------- Cylinder container --------
CollisionManifold Contain(const WorldSphere& a, const WorldCylinder& container);
CollisionManifold Contain(const WorldCapsule& a, const WorldCylinder& container);
CollisionManifold Contain(const WorldCylinder& a, const WorldCylinder& container);
CollisionManifold Contain(const WorldOBB& a, const WorldCylinder& container);