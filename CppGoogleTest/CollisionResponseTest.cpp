#include "pch.h"
#include "PhysicsObject.h"
#include "Sphere.h"
#include "Plane.h"
#include <glm/glm.hpp>
#include <glm/gtc/epsilon.hpp>

// Helper function to check if two vectors are approximately equal
bool VectorsEqual(const glm::vec3& a, const glm::vec3& b, float epsilon = 0.001f)
{
    return glm::all(glm::epsilonEqual(a, b, epsilon));
}

// ===== TEST 1: Ball bounces straight down off ground =====
TEST(CollisionResponse, SpherePlaneHeadOnBounce)
{
    // Setup: ball falling straight down with velocity -10 m/s in Y
    glm::vec3 ballVel(0.0f, -10.0f, 0.0f);

    // Ground plane: normal points UP (+Y)
    glm::vec3 planeNormal(0.0f, 1.0f, 0.0f);

    // Simulate collision response
    float restitution = 0.8f;  // 80% energy retained
    float vDotN = glm::dot(ballVel, planeNormal);  // Should be -10

    // Apply impulse formula: J = -(1 + e) * (v · n) * n
    glm::vec3 impulse = -(1.0f + restitution) * vDotN * planeNormal;
    glm::vec3 newVel = ballVel + impulse;  // For unit mass

    // Expected: velocity reverses and reduces to 80% magnitude
    // old: (0, -10, 0)  -> new: (0, 8, 0)
    EXPECT_TRUE(VectorsEqual(newVel, glm::vec3(0.0f, 8.0f, 0.0f)))
        << "Velocity was: (" << newVel.x << ", " << newVel.y << ", " << newVel.z << ")";
}

// ===== TEST 2: Ball bounces at 45° angle =====
TEST(CollisionResponse, SphereePlaneGlancingBounce)
{
    // Ball moving at 45° into ground: velocity (5, -5, 0)
    glm::vec3 ballVel(5.0f, -5.0f, 0.0f);

    // Ground plane: normal points UP
    glm::vec3 planeNormal(0.0f, 1.0f, 0.0f);

    float restitution = 0.8f;
    float vDotN = glm::dot(ballVel, planeNormal);  // = -5

    glm::vec3 impulse = -(1.0f + restitution) * vDotN * planeNormal;
    glm::vec3 newVel = ballVel + impulse;

    // Expected:
    // - X component stays the same (no horizontal impulse)
    // - Y component bounces: -5 -> 4 (80% restitution)
    EXPECT_FLOAT_EQ(newVel.x, 5.0f) << "X velocity should not change";
    EXPECT_FLOAT_EQ(newVel.y, 4.0f) << "Y velocity should bounce to 4.0";
    EXPECT_FLOAT_EQ(newVel.z, 0.0f);
}

// ===== TEST 3: Ball moving away from plane (no collision) =====
TEST(CollisionResponse, SpherePlaneMovingAway)
{
    // Ball moving AWAY from ground (velocity positive Y)
    glm::vec3 ballVel(0.0f, 5.0f, 0.0f);

    glm::vec3 planeNormal(0.0f, 1.0f, 0.0f);

    float vDotN = glm::dot(ballVel, planeNormal);  // = 5.0 (POSITIVE)

    // Should NOT apply impulse if vDotN >= 0
    // (already separating from plane)
    EXPECT_GE(vDotN, 0.0f) << "Ball should be moving away from plane";

    // Velocity should NOT change
    EXPECT_TRUE(VectorsEqual(ballVel, glm::vec3(0.0f, 5.0f, 0.0f)));
}

// ===== TEST 4: Perfect elastic collision (restitution = 1.0) =====
TEST(CollisionResponse, SpherePlaneElasticBounce)
{
    // Ball falling at 10 m/s
    glm::vec3 ballVel(0.0f, -10.0f, 0.0f);
    glm::vec3 planeNormal(0.0f, 1.0f, 0.0f);

    float restitution = 1.0f;  // Perfect elastic
    float vDotN = glm::dot(ballVel, planeNormal);

    glm::vec3 impulse = -(1.0f + restitution) * vDotN * planeNormal;
    glm::vec3 newVel = ballVel + impulse;

    // Expected: velocity completely reverses (no energy loss)
    // old: (0, -10, 0) -> new: (0, 10, 0)
    EXPECT_TRUE(VectorsEqual(newVel, glm::vec3(0.0f, 10.0f, 0.0f)))
        << "With restitution=1.0, should reverse completely";
}

// ===== TEST 5: Perfectly inelastic collision (restitution = 0) =====
TEST(CollisionResponse, SphereplanePerfectlyInelasticBounce)
{
    glm::vec3 ballVel(0.0f, -10.0f, 0.0f);
    glm::vec3 planeNormal(0.0f, 1.0f, 0.0f);

    float restitution = 0.0f;  // No bounce
    float vDotN = glm::dot(ballVel, planeNormal);

    glm::vec3 impulse = -(1.0f + restitution) * vDotN * planeNormal;
    glm::vec3 newVel = ballVel + impulse;

    // Expected: velocity in normal direction becomes 0
    // old: (0, -10, 0) -> new: (0, 0, 0)
    EXPECT_TRUE(VectorsEqual(newVel, glm::vec3(0.0f, 0.0f, 0.0f)))
        << "With restitution=0, should stop completely";
}

// ===== TEST 6: Angled plane collision =====
TEST(CollisionResponse, SphereAngledPlaneBounceWithHorizontalVelocity)
{
    // Ball moving diagonally (5 right, -10 down)
    glm::vec3 ballVel(5.0f, -10.0f, 0.0f);

    // Plane angled at 45°: normal = (1, 1, 0) normalized
    glm::vec3 planeNormal = glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f));

    float restitution = 0.8f;
    float vDotN = glm::dot(ballVel, planeNormal);

    glm::vec3 impulse = -(1.0f + restitution) * vDotN * planeNormal;
    glm::vec3 newVel = ballVel + impulse;

    // After bounce, should be moving away from plane
    float vDotN_new = glm::dot(newVel, planeNormal);

    EXPECT_GT(vDotN_new, 0.0f)
        << "Ball should bounce away from angled plane. vDotN_new = " << vDotN_new
        << ", newVel = (" << newVel.x << ", " << newVel.y << ", " << newVel.z << ")";
}
// ===== TEST 7: Moving sphere hits fixed sphere =====
TEST(CollisionResponse, MovingSphereHitsFixedSphere)
{
    // Moving sphere: velocity (5, 0, 0), mass = 1
    glm::vec3 vMoving(5.0f, 0.0f, 0.0f);
    float mMoving = 1.0f;

    // Fixed sphere: velocity (0, 0, 0), mass = 0 (infinite)
    glm::vec3 vFixed(0.0f, 0.0f, 0.0f);
    float mFixed = 0.0f;

    // Collision normal: pointing from moving to fixed (1, 0, 0)
    glm::vec3 n(1.0f, 0.0f, 0.0f);

    // Relative velocity
    glm::vec3 vRel = vMoving - vFixed;  // (5, 0, 0)
    float vRelDotN = glm::dot(vRel, n);  // 5

    float restitution = 0.8f;

    // For collision with fixed object (infinite mass):
    // impulse magnitude = -(1 + e) * vRel·n / (1/m_moving + 1/m_fixed)
    // But 1/m_fixed = 1/0 = infinity, so denominator = infinity
    // This means we should just use: impulse = -(1 + e) * vRel·n * n for the moving object

    glm::vec3 impulse = -(1.0f + restitution) * vRelDotN * n;
    glm::vec3 newVelMoving = vMoving + impulse / mMoving;  // divide by mass

    // Expected: moving sphere bounces back
    // old: (5, 0, 0) -> new: (-4, 0, 0)  (reversed and reduced by restitution)
    EXPECT_LT(newVelMoving.x, 0.0f) << "Moving sphere should bounce back (negative X)";
    EXPECT_FLOAT_EQ(newVelMoving.x, -4.0f) << "Should be -4.0 with restitution 0.8";
}

// ===== TEST 8: Two equal mass spheres - head on collision =====
TEST(CollisionResponse, TwoEqualMassSpheresHeadOn)
{
    // Sphere A: moving RIGHT at 5 m/s
    glm::vec3 vA(5.0f, 0.0f, 0.0f);
    float mA = 1.0f;

    // Sphere B: stationary
    glm::vec3 vB(0.0f, 0.0f, 0.0f);
    float mB = 1.0f;

    // Collision normal (from A to B)
    glm::vec3 n(1.0f, 0.0f, 0.0f);

    glm::vec3 vRel = vA - vB;  // (5, 0, 0)
    float vRelDotN = glm::dot(vRel, n);  // 5

    float restitution = 0.8f;

    // Impulse for two bodies
    float impulseMag = -(1.0f + restitution) * vRelDotN / (1.0f / mA + 1.0f / mB);
    glm::vec3 impulse = impulseMag * n;

    glm::vec3 newVelA = vA + impulse / mA;
    glm::vec3 newVelB = vB - impulse / mB;  // Opposite impulse

    // Expected: A slows down, B speeds up
    EXPECT_LT(newVelA.x, vA.x) << "A should slow down";
    EXPECT_GT(newVelB.x, vB.x) << "B should speed up";
    EXPECT_GT(newVelB.x, 0.0f) << "B should move in positive X";
}
TEST(Q2_SameMassBallCollisions, BothBallsMovingHeadOn)
{
    // Ball A: moving RIGHT at 5 m/s
    glm::vec3 vA(5.0f, 0.0f, 0.0f);
    float mA = 1.0f;

    // Ball B: moving LEFT at 3 m/s
    glm::vec3 vB(-3.0f, 0.0f, 0.0f);
    float mB = 1.0f;

    // Collision normal (from A to B, pointing right)
    glm::vec3 n(1.0f, 0.0f, 0.0f);

    glm::vec3 vRel = vA - vB;  // (8, 0, 0)
    float vRelDotN = glm::dot(vRel, n);  // 8

    float restitution = 0.8f;

    // Impulse calculation for equal mass
    float impulseMag = -(1.0f + restitution) * vRelDotN / (1.0f / mA + 1.0f / mB);
    glm::vec3 impulse = impulseMag * n;

    glm::vec3 newVelA = vA + impulse / mA;
    glm::vec3 newVelB = vB - impulse / mB;

    // Expected behavior:
    // - Both balls should exchange direction (roughly)
    // - A should slow down or reverse
    // - B should slow down or reverse

    EXPECT_LT(newVelA.x, vA.x) << "Ball A should slow down or reverse";
    EXPECT_GT(newVelB.x, vB.x) << "Ball B should slow down or reverse";
}

// ===== TEST Q2.2: One ball moving, one stationary - head on =====
TEST(Q2_SameMassBallCollisions, OneBallMovingHeadOn)
{
    // Ball A: moving RIGHT at 6 m/s
    glm::vec3 vA(6.0f, 0.0f, 0.0f);
    float mA = 1.0f;

    // Ball B: stationary
    glm::vec3 vB(0.0f, 0.0f, 0.0f);
    float mB = 1.0f;

    glm::vec3 n(1.0f, 0.0f, 0.0f);
    glm::vec3 vRel = vA - vB;
    float vRelDotN = glm::dot(vRel, n);  // 6

    float restitution = 0.8f;
    float impulseMag = -(1.0f + restitution) * vRelDotN / (1.0f / mA + 1.0f / mB);
    glm::vec3 impulse = impulseMag * n;

    glm::vec3 newVelA = vA + impulse / mA;
    glm::vec3 newVelB = vB - impulse / mB;

    // Expected with equal mass and restitution 0.8:
    // - A should slow down but keep moving forward
    // - B should move forward faster than A

    EXPECT_LT(newVelA.x, vA.x) << "Ball A should slow down";
    EXPECT_GT(newVelB.x, newVelA.x) << "Ball B should be faster than A";
    EXPECT_GT(newVelB.x, 0.0f) << "Ball B should move forward";

    // Correct expected values:
    // impulseMag = -(1.8) * 6 / 2 = -5.4
    // newVelA = 6 + (-5.4) = 0.6
    // newVelB = 0 - (-5.4) = 5.4
    EXPECT_NEAR(newVelA.x, 0.6f, 0.001f) << "A should be 0.6 m/s";
    EXPECT_NEAR(newVelB.x, 5.4f, 0.001f) << "B should be 5.4 m/s";
}

// ===== TEST Q2.3: Both balls moving same direction, A faster =====
TEST(Q2_SameMassBallCollisions, BothMovingSameDirectionAFaster)
{
    // Ball A: moving RIGHT at 8 m/s (faster)
    glm::vec3 vA(8.0f, 0.0f, 0.0f);
    float mA = 1.0f;

    // Ball B: moving RIGHT at 3 m/s (slower)
    glm::vec3 vB(3.0f, 0.0f, 0.0f);
    float mB = 1.0f;

    glm::vec3 n(1.0f, 0.0f, 0.0f);
    glm::vec3 vRel = vA - vB;  // (5, 0, 0) - A catching up to B
    float vRelDotN = glm::dot(vRel, n);  // 5

    float restitution = 0.8f;
    float impulseMag = -(1.0f + restitution) * vRelDotN / (1.0f / mA + 1.0f / mB);
    glm::vec3 impulse = impulseMag * n;

    glm::vec3 newVelA = vA + impulse / mA;
    glm::vec3 newVelB = vB - impulse / mB;

    // Expected:
    // - A should slow down (impulse opposes its motion)
    // - B should speed up (impulse accelerates it forward)
    // - They should exchange momentum

    EXPECT_LT(newVelA.x, vA.x) << "Ball A should slow down";
    EXPECT_GT(newVelB.x, vB.x) << "Ball B should speed up";
}

// ===== TEST Q2.4: Elastic collision (restitution = 1.0) =====
TEST(Q2_SameMassBallCollisions, ElasticCollisionHeadOn)
{
    glm::vec3 vA(5.0f, 0.0f, 0.0f);
    float mA = 1.0f;

    glm::vec3 vB(0.0f, 0.0f, 0.0f);
    float mB = 1.0f;

    glm::vec3 n(1.0f, 0.0f, 0.0f);
    glm::vec3 vRel = vA - vB;
    float vRelDotN = glm::dot(vRel, n);

    float restitution = 1.0f;  // Perfect elastic
    float impulseMag = -(1.0f + restitution) * vRelDotN / (1.0f / mA + 1.0f / mB);
    glm::vec3 impulse = impulseMag * n;

    glm::vec3 newVelA = vA + impulse / mA;
    glm::vec3 newVelB = vB - impulse / mB;

    // Expected: complete velocity exchange
    // A should stop, B should have A's original velocity
    EXPECT_NEAR(newVelA.x, 0.0f, 0.001f) << "A should stop";
    EXPECT_NEAR(newVelB.x, 5.0f, 0.001f) << "B should get A's velocity";
}

// ===== TEST Q2.5: Perfectly inelastic (restitution = 0) =====
TEST(Q2_SameMassBallCollisions, PerfectlyInelasticHeadOn)
{
    glm::vec3 vA(5.0f, 0.0f, 0.0f);
    float mA = 1.0f;

    glm::vec3 vB(-3.0f, 0.0f, 0.0f);
    float mB = 1.0f;

    glm::vec3 n(1.0f, 0.0f, 0.0f);
    glm::vec3 vRel = vA - vB;  // (8, 0, 0)
    float vRelDotN = glm::dot(vRel, n);

    float restitution = 0.0f;  // Perfectly inelastic
    float impulseMag = -(1.0f + restitution) * vRelDotN / (1.0f / mA + 1.0f / mB);
    glm::vec3 impulse = impulseMag * n;

    glm::vec3 newVelA = vA + impulse / mA;
    glm::vec3 newVelB = vB - impulse / mB;

    // Expected: they stick together (same velocity)
    EXPECT_NEAR(newVelA.x, newVelB.x, 0.001f) << "Both should have same velocity after perfectly inelastic collision";

    // Should conserve momentum: (5*1 + -3*1) / 2 = 1
    float expectedVel = (5.0f - 3.0f) / 2.0f;  // = 1.0
    EXPECT_NEAR(newVelA.x, expectedVel, 0.001f) << "Should conserve momentum";
}
// ===== TEST Q3.1: Heavy ball hits light ball (head on) =====
TEST(Q3_DifferentMassBallCollisions, HeavyHitsLight)
{
    // Ball 1: Heavy (mass 2), moving RIGHT at 4 m/s
    float m1 = 2.0f;
    glm::vec3 v1(4.0f, 0.0f, 0.0f);

    // Ball 2: Light (mass 1), stationary
    float m2 = 1.0f;
    glm::vec3 v2(0.0f, 0.0f, 0.0f);

    glm::vec3 n(1.0f, 0.0f, 0.0f);
    glm::vec3 vRel = v1 - v2;
    float vRelDotN = glm::dot(vRel, n);

    float restitution = 0.8f;
    float impulseMag = -(1.0f + restitution) * vRelDotN / (1.0f / m1 + 1.0f / m2);
    glm::vec3 impulse = impulseMag * n;

    glm::vec3 newV1 = v1 + impulse / m1;
    glm::vec3 newV2 = v2 - impulse / m2;

    // Expected behavior:
    // - Heavy ball should slow down
    // - Light ball should move FASTER than heavy ball (gets all the momentum)

    EXPECT_LT(newV1.x, v1.x) << "Heavy ball should slow down";
    EXPECT_GT(newV2.x, v2.x) << "Light ball should speed up";
    EXPECT_LT(newV1.x, newV2.x) << "Light ball should be FASTER than heavy ball!";

    // Verify values
    EXPECT_NEAR(newV1.x, 1.6f, 0.001f) << "Heavy ball should be ~1.6 m/s";
    EXPECT_NEAR(newV2.x, 4.8f, 0.001f) << "Light ball should be ~4.8 m/s";
}

// ===== TEST Q3.2: Light ball hits heavy ball (head on) =====
TEST(Q3_DifferentMassBallCollisions, LightHitsHeavy)
{
    // Ball 1: Light (mass 1), moving RIGHT at 6 m/s
    float m1 = 1.0f;
    glm::vec3 v1(6.0f, 0.0f, 0.0f);

    // Ball 2: Heavy (mass 3), stationary
    float m2 = 3.0f;
    glm::vec3 v2(0.0f, 0.0f, 0.0f);

    glm::vec3 n(1.0f, 0.0f, 0.0f);
    glm::vec3 vRel = v1 - v2;
    float vRelDotN = glm::dot(vRel, n);

    float restitution = 0.8f;
    float impulseMag = -(1.0f + restitution) * vRelDotN / (1.0f / m1 + 1.0f / m2);
    glm::vec3 impulse = impulseMag * n;

    glm::vec3 newV1 = v1 + impulse / m1;
    glm::vec3 newV2 = v2 - impulse / m2;

    // Expected behavior:
    // - Light ball should BOUNCE BACK (reverse direction, negative velocity)
    // - Heavy ball should move forward but slowly
    // - Light ball velocity will be negative, heavy ball positive

    EXPECT_LT(newV1.x, 0.0f) << "Light ball should bounce back (negative velocity)";
    EXPECT_GT(newV2.x, 0.0f) << "Heavy ball should move forward (positive velocity)";
    EXPECT_LT(newV1.x, newV2.x) << "Light ball velocity should be less (more negative) than heavy ball";

    // Verify values
    EXPECT_NEAR(newV1.x, -2.1f, 0.1f) << "Light ball should bounce to ~-2.1 m/s";
    EXPECT_NEAR(newV2.x, 2.7f, 0.1f) << "Heavy ball should move to ~2.7 m/s";
}

// ===== TEST Q3.3: Both moving, different masses, opposite directions =====
TEST(Q3_DifferentMassBallCollisions, BothMovingOppositeDifferentMass)
{
    // Ball 1: mass 2, moving RIGHT at 5 m/s
    float m1 = 2.0f;
    glm::vec3 v1(5.0f, 0.0f, 0.0f);

    // Ball 2: mass 1, moving LEFT at 3 m/s
    float m2 = 1.0f;
    glm::vec3 v2(-3.0f, 0.0f, 0.0f);

    glm::vec3 n(1.0f, 0.0f, 0.0f);
    glm::vec3 vRel = v1 - v2;  // (8, 0, 0)
    float vRelDotN = glm::dot(vRel, n);  // 8

    float restitution = 0.8f;
    float impulseMag = -(1.0f + restitution) * vRelDotN / (1.0f / m1 + 1.0f / m2);
    glm::vec3 impulse = impulseMag * n;

    glm::vec3 newV1 = v1 + impulse / m1;
    glm::vec3 newV2 = v2 - impulse / m2;

    // Expected:
    // - Both balls should change direction based on their mass

    EXPECT_LT(newV1.x, v1.x) << "Ball 1 should slow down";
    EXPECT_GT(newV2.x, v2.x) << "Ball 2 should speed up (less negative)";
}

// ===== TEST Q3.4: Energy conservation =====
TEST(Q3_DifferentMassBallCollisions, EnergyConservation)
{
    float m1 = 2.0f;
    glm::vec3 v1(4.0f, 0.0f, 0.0f);

    float m2 = 1.0f;
    glm::vec3 v2(0.0f, 0.0f, 0.0f);

    glm::vec3 n(1.0f, 0.0f, 0.0f);
    glm::vec3 vRel = v1 - v2;
    float vRelDotN = glm::dot(vRel, n);

    float restitution = 0.8f;
    float impulseMag = -(1.0f + restitution) * vRelDotN / (1.0f / m1 + 1.0f / m2);
    glm::vec3 impulse = impulseMag * n;

    glm::vec3 newV1 = v1 + impulse / m1;
    glm::vec3 newV2 = v2 - impulse / m2;

    // Kinetic energy before
    float KE_before = 0.5f * m1 * v1.x * v1.x + 0.5f * m2 * v2.x * v2.x;
    // KE_before = 0.5 * 2 * 16 + 0 = 16

    // Kinetic energy after
    float KE_after = 0.5f * m1 * newV1.x * newV1.x + 0.5f * m2 * newV2.x * newV2.x;
    // KE_after = 0.5 * 2 * 1.6^2 + 0.5 * 1 * 4.8^2 = 2.56 + 11.52 = 14.08

    // With restitution < 1, energy should decrease
    EXPECT_LE(KE_after, KE_before) << "Kinetic energy should decrease with inelastic collision";
    EXPECT_FLOAT_EQ(KE_before, 16.0f);
    EXPECT_NEAR(KE_after, 14.08f, 0.1f);
}

// ===== TEST Q3.5: Elastic collision (restitution = 1.0) different mass =====
TEST(Q3_DifferentMassBallCollisions, ElasticDifferentMass)
{
    float m1 = 2.0f;
    glm::vec3 v1(4.0f, 0.0f, 0.0f);

    float m2 = 1.0f;
    glm::vec3 v2(0.0f, 0.0f, 0.0f);

    glm::vec3 n(1.0f, 0.0f, 0.0f);
    glm::vec3 vRel = v1 - v2;
    float vRelDotN = glm::dot(vRel, n);

    float restitution = 1.0f;  // Elastic
    float impulseMag = -(1.0f + restitution) * vRelDotN / (1.0f / m1 + 1.0f / m2);
    glm::vec3 impulse = impulseMag * n;

    glm::vec3 newV1 = v1 + impulse / m1;
    glm::vec3 newV2 = v2 - impulse / m2;

    // Momentum before and after
    float momentum_before = m1 * v1.x + m2 * v2.x;
    float momentum_after = m1 * newV1.x + m2 * newV2.x;

    EXPECT_NEAR(momentum_before, momentum_after, 0.001f) << "Momentum should be conserved";
}

// ===== TEST Q3.6: Very different masses (100:1 ratio) =====
TEST(Q3_DifferentMassBallCollisions, ExtremeMassRatio)
{
    float m1 = 100.0f;  // Bowling ball
    glm::vec3 v1(2.0f, 0.0f, 0.0f);

    float m2 = 1.0f;    // Marble
    glm::vec3 v2(0.0f, 0.0f, 0.0f);

    glm::vec3 n(1.0f, 0.0f, 0.0f);
    glm::vec3 vRel = v1 - v2;
    float vRelDotN = glm::dot(vRel, n);

    float restitution = 0.8f;
    float impulseMag = -(1.0f + restitution) * vRelDotN / (1.0f / m1 + 1.0f / m2);
    glm::vec3 impulse = impulseMag * n;

    glm::vec3 newV1 = v1 + impulse / m1;
    glm::vec3 newV2 = v2 - impulse / m2;

    // Expected:
    // - Heavy ball barely slows down (large mass, small force effect)
    // - Light ball shoots forward VERY fast

    EXPECT_NEAR(newV1.x, v1.x, 0.05f) << "Heavy ball should barely slow down";
    EXPECT_GT(newV2.x, 3.0f) << "Light ball should move much faster than heavy ball";
}