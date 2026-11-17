#include <gtest/gtest.h>

#include "ecs/World.hpp"
#include "physics/Collision.hpp"
#include "physics/Constraint.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <cmath>

namespace simforge {

TEST(AdvancedPhysics, RigidBodyInertiaCalculation) {
  RigidBodyComponent rb;
  rb.inverseMass = 1.0f / 6.0f;  // 6 kg

  // Sphere: I = 0.4 * m * r^2 = 0.4 * 6 * 1^2 = 2.4
  rb.computeInertia(ColliderShape::Sphere, 1.0f, glm::vec3(0.0f));
  EXPECT_NEAR(rb.inverseInertiaTensorBody[0][0], 1.0f / 2.4f, 1e-4f);
  EXPECT_NEAR(rb.inverseInertiaTensorBody[1][1], 1.0f / 2.4f, 1e-4f);
  EXPECT_NEAR(rb.inverseInertiaTensorBody[2][2], 1.0f / 2.4f, 1e-4f);

  // Box: halfExtents = (1, 1, 1) => I_xx = (1/3) * 6 * (1 + 1) = 4.0
  rb.computeInertia(ColliderShape::Box, 0.0f, glm::vec3(1.0f));
  EXPECT_NEAR(rb.inverseInertiaTensorBody[0][0], 1.0f / 4.0f, 1e-4f);
  EXPECT_NEAR(rb.inverseInertiaTensorBody[1][1], 1.0f / 4.0f, 1e-4f);
  EXPECT_NEAR(rb.inverseInertiaTensorBody[2][2], 1.0f / 4.0f, 1e-4f);
}

TEST(AdvancedPhysics, QuaternionAngularIntegration) {
  RigidBodyComponent rb;
  TransformComponent t;
  rb.inverseMass = 1.0f;
  rb.isStatic = false;
  rb.useGravity = false;
  rb.angularDamping = 0.0f;
  rb.linearDamping = 0.0f;

  // Spin around Y axis at 1 rad/s
  rb.angularVelocity = glm::vec3(0.0f, 1.0f, 0.0f);
  t.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

  float dt = 0.1f;
  integrateRigidBody(rb, t, glm::vec3(0.0f), dt);

  // Expected rotation around Y by angle = w * dt = 0.1 rad
  // Half angle = 0.05 rad => w = cos(0.05) ~ 0.99875, y = sin(0.05) ~ 0.049979
  EXPECT_NEAR(t.rotation.w, std::cos(0.05f), 1e-3f);
  EXPECT_NEAR(t.rotation.y, std::sin(0.05f), 1e-3f);
  EXPECT_NEAR(t.rotation.x, 0.0f, 1e-5f);
  EXPECT_NEAR(t.rotation.z, 0.0f, 1e-5f);
}

TEST(AdvancedPhysics, OBBCollisionRotatedBoxes) {
  TransformComponent a;
  ColliderComponent ca;
  a.position = glm::vec3(0.0f, 0.0f, 0.0f);
  a.rotation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  ca.shape = ColliderShape::Box;
  ca.halfExtents = glm::vec3(1.0f, 1.0f, 1.0f);

  TransformComponent b;
  ColliderComponent cb;
  b.position = glm::vec3(2.0f, 0.0f, 0.0f);
  b.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  cb.shape = ColliderShape::Box;
  cb.halfExtents = glm::vec3(1.0f, 1.0f, 1.0f);

  // A has corner extending out to sqrt(2) ~ 1.414 on X axis.
  // B starts at 2.0 with half-width 1.0 (from 1.0 to 3.0 on X).
  // 1.414 > 1.0 => Overlap occurs!
  auto hit = collideBoxBox(a, ca, b, cb);
  ASSERT_TRUE(hit.has_value());
  EXPECT_GT(hit->penetration, 0.0f);

  // Move B far away: no collision
  b.position = glm::vec3(5.0f, 0.0f, 0.0f);
  auto noHit = collideBoxBox(a, ca, b, cb);
  EXPECT_FALSE(noHit.has_value());
}

TEST(AdvancedPhysics, CoulombFrictionImpulse) {
  RigidBodyComponent bodyA;
  TransformComponent tA;
  bodyA.inverseMass = 0.0f;  // Static ground
  bodyA.isStatic = true;
  bodyA.staticFriction = 0.6f;
  bodyA.dynamicFriction = 0.4f;
  tA.position = glm::vec3(0.0f, 0.0f, 0.0f);

  RigidBodyComponent bodyB;
  TransformComponent tB;
  bodyB.inverseMass = 1.0f;  // 1 kg
  bodyB.isStatic = false;
  bodyB.staticFriction = 0.6f;
  bodyB.dynamicFriction = 0.4f;
  bodyB.restitution = 0.0f;
  bodyB.computeInertia(ColliderShape::Box, 0.0f, glm::vec3(0.5f));
  tB.position = glm::vec3(0.0f, 1.0f, 0.0f);

  // Moving downwards and sideways along X
  bodyB.velocity = glm::vec3(5.0f, -2.0f, 0.0f);

  ContactManifold contact;
  contact.normal = glm::vec3(0.0f, 1.0f, 0.0f);
  contact.penetration = 0.05f;
  contact.point = glm::vec3(0.0f, 0.5f, 0.0f);

  resolveContact(bodyA, tA, bodyB, tB, contact);

  // Vertical velocity should be resolved (no longer moving downwards)
  EXPECT_GE(bodyB.velocity.y, -1e-4f);
  // Tangential velocity should be reduced by friction
  EXPECT_LT(bodyB.velocity.x, 5.0f);
  EXPECT_GT(bodyB.velocity.x, 0.0f);
}

TEST(AdvancedPhysics, RaycastIntersections) {
  // Ray pointing along -Z towards a sphere at (0, 0, -5)
  Ray ray;
  ray.origin = glm::vec3(0.0f, 0.0f, 0.0f);
  ray.direction = glm::vec3(0.0f, 0.0f, -1.0f);

  float tHit = 0.0f;
  glm::vec3 hitNorm(0.0f);
  bool hitSphere = intersectRaySphere(ray, glm::vec3(0.0f, 0.0f, -5.0f), 1.0f, tHit, hitNorm);
  EXPECT_TRUE(hitSphere);
  EXPECT_NEAR(tHit, 4.0f, 1e-4f);  // Origin at 0, sphere surface at -4
  EXPECT_NEAR(hitNorm.z, 1.0f, 1e-4f);

  // Ray vs OBB
  bool hitOBB = intersectRayOBB(ray, glm::vec3(0.0f, 0.0f, -5.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                                glm::vec3(1.0f, 1.0f, 1.0f), tHit, hitNorm);
  EXPECT_TRUE(hitOBB);
  EXPECT_NEAR(tHit, 4.0f, 1e-4f);
  EXPECT_NEAR(hitNorm.z, 1.0f, 1e-4f);

  // Ray misses when offset
  Ray missRay;
  missRay.origin = glm::vec3(10.0f, 10.0f, 0.0f);
  missRay.direction = glm::vec3(0.0f, 0.0f, -1.0f);
  EXPECT_FALSE(intersectRaySphere(missRay, glm::vec3(0.0f, 0.0f, -5.0f), 1.0f, tHit, hitNorm));
  EXPECT_FALSE(intersectRayOBB(missRay, glm::vec3(0.0f, 0.0f, -5.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                               glm::vec3(1.0f, 1.0f, 1.0f), tHit, hitNorm));
}

TEST(AdvancedPhysics, DistanceConstraintSolvesLength) {
  World world;
  ConstraintSolver solver;

  Entity eA = world.createEntity("anchorA");
  auto* tA = world.transform(eA);
  auto* rbA = world.rigidBody(eA);
  tA->position = glm::vec3(0.0f, 5.0f, 0.0f);
  rbA->isStatic = true;
  rbA->inverseMass = 0.0f;

  Entity eB = world.createEntity("bobB");
  auto* tB = world.transform(eB);
  auto* rbB = world.rigidBody(eB);
  tB->position = glm::vec3(0.0f, 2.0f, 0.0f);  // Distance = 3.0
  rbB->isStatic = false;
  rbB->inverseMass = 1.0f;
  rbB->velocity = glm::vec3(0.0f, -4.0f, 0.0f);  // Moving away downwards

  DistanceJoint joint;
  joint.entityA = eA.id;
  joint.entityB = eB.id;
  joint.targetDistance = 3.0f;
  joint.stiffness = 1.0f;

  std::vector<DistanceJoint> joints = {joint};

  // Velocity constraint should eliminate downward velocity
  solver.solveVelocityConstraints(world, joints, 1.0f / 60.0f);
  EXPECT_NEAR(rbB->velocity.y, 0.0f, 0.1f);
}

}  // namespace simforge
