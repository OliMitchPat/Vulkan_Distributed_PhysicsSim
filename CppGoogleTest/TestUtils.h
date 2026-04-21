#pragma once
#include <gtest/gtest.h>
#include <glm/glm.hpp>
#include <cmath>

inline void ExpectVec3Near(const glm::vec3& a, const glm::vec3& b, float eps, const char* msg = "")
{
    EXPECT_NEAR(a.x, b.x, eps) << msg;
    EXPECT_NEAR(a.y, b.y, eps) << msg;
    EXPECT_NEAR(a.z, b.z, eps) << msg;
}

inline void ExpectUnitOrFallbackNormal(const glm::vec3& n, float eps = 1e-4f)
{
    // We expect unit normals for all hit manifolds in your code.
    const float len = std::sqrt(glm::dot(n, n));
    EXPECT_NEAR(len, 1.0f, eps) << "Normal must be unit length";
}

template <typename ManifoldT>
inline void ExpectManifoldHitInvariants(const ManifoldT& m, float eps = 1e-4f)
{
    ASSERT_TRUE(m.hit);
    ExpectUnitOrFallbackNormal(m.normal, eps);
    EXPECT_GE(m.penetration, 0.0f);
}

inline void ExpectManifoldHitBasic(const CollisionManifold& m)
{
    ASSERT_TRUE(m.hit);
    EXPECT_GE(m.penetration, 0.0f);
    EXPECT_TRUE(std::isfinite(m.normal.x));
    EXPECT_TRUE(std::isfinite(m.normal.y));
    EXPECT_TRUE(std::isfinite(m.normal.z));
}