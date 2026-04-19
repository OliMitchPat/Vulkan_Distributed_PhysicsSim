#pragma once
#include "WorldShapes.h"
#include "CollisionManifold.h"

// Sphere vs ...
CollisionManifold Intersect(const WorldSphere& a, const WorldSphere& b);
CollisionManifold Intersect(const WorldSphere& s, const WorldPlane& p);
CollisionManifold Intersect(const WorldSphere& s, const WorldCapsule& c);
CollisionManifold Intersect(const WorldSphere& s, const WorldCylinder& c);
CollisionManifold Intersect(const WorldSphere& s, const WorldOBB& b);

// Capsule vs ...
CollisionManifold Intersect(const WorldCapsule& a, const WorldCapsule& b);
CollisionManifold Intersect(const WorldCapsule& c, const WorldPlane& p);
CollisionManifold Intersect(const WorldCapsule& c, const WorldOBB& b);
CollisionManifold Intersect(const WorldCapsule& c, const WorldCylinder& y);

// OBB vs ...
CollisionManifold Intersect(const WorldOBB& a, const WorldPlane& p);
CollisionManifold Intersect(const WorldOBB& a, const WorldOBB& b);

// Cylinder vs ...
CollisionManifold Intersect(const WorldCylinder& c, const WorldPlane& p);
CollisionManifold Intersect(const WorldCylinder& c, const WorldOBB& b);
CollisionManifold Intersect(const WorldCylinder& a, const WorldCylinder& b);