#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "physics/ContactSolver.hpp"
#include "physics/PhysicsWorld.hpp"

using namespace simforge;

namespace {
Entity ground(World& world) {
  const auto entity = world.createEntity();
  world.transform(entity)->position.y = -0.5f;
  world.collider(entity)->shape = ColliderShape::Box;
  world.collider(entity)->halfExtents = {5.0f, 0.5f, 5.0f};
  world.rigidBody(entity)->isStatic = true;
  world.rigidBody(entity)->inverseMass = 0.0f;
  return entity;
}
void boxBody(World& world, Entity entity, float height) {
  world.transform(entity)->position.y = height;
  world.collider(entity)->shape = ColliderShape::Box;
  auto* body = world.rigidBody(entity);
  body->restitution = 0.0f;
  body->linearDamping = 0.0f;
  body->angularDamping = 0.0f;
  body->computeInertia(ColliderShape::Box, 0.0f, glm::vec3(0.5f));
}
}  // namespace

TEST(ReferencePhysics, FreeFallMatchesAnalyticVelocityAndBoundedPositionError) {
  World world;
  const auto entity = world.createEntity();
  world.transform(entity)->position.y = 20.0f;
  world.rigidBody(entity)->linearDamping = 0.0f;
  PhysicsWorld physics;
  for (int i = 0; i < 120; ++i) physics.stepFixed(world);
  const double t = physics.simulationTime();
  EXPECT_NEAR(world.rigidBody(entity)->velocity.y, -9.81 * t, 0.0001);
  EXPECT_NEAR(world.transform(entity)->position.y, 20.0 - 0.5 * 9.81 * t * t, 0.042);
}

TEST(ReferencePhysics, ElasticCollisionConservesLinearMomentumAndKineticEnergy) {
  World world;
  const auto a = world.createEntity(), b = world.createEntity();
  world.transform(a)->position.x = -0.49f;
  world.transform(b)->position.x = 0.49f;
  for (const auto entity : {a, b}) {
    auto* body = world.rigidBody(entity);
    body->useGravity = false;
    body->restitution = 1.0f;
    body->linearDamping = 0.0f;
    body->staticFriction = body->dynamicFriction = 0.0f;
    body->computeInertia(ColliderShape::Sphere, 0.5f, glm::vec3(0.0f));
  }
  world.rigidBody(a)->velocity.x = 2.0f;
  world.rigidBody(b)->velocity.x = -1.0f;
  PhysicsWorld physics;
  physics.stepFixed(world);
  const float va = world.rigidBody(a)->velocity.x, vb = world.rigidBody(b)->velocity.x;
  EXPECT_NEAR(va, -1.0f, 1e-5f);
  EXPECT_NEAR(vb, 2.0f, 1e-5f);
  EXPECT_NEAR(va + vb, 1.0f, 1e-5f);
  EXPECT_NEAR(0.5f * (va * va + vb * vb), 2.5f, 1e-5f);
}

TEST(ReferencePhysics, RestingBoxDoesNotAcquireArtificialSpin) {
  World world;
  ground(world);
  const auto box = world.createEntity();
  boxBody(world, box, 0.5f);
  PhysicsWorld physics;
  for (int i = 0; i < 240; ++i) physics.stepFixed(world);
  EXPECT_NEAR(world.transform(box)->position.y, 0.5f, 0.005f);
  EXPECT_LT(glm::length(world.rigidBody(box)->angularVelocity), 0.01f);
  EXPECT_GT(std::abs(world.transform(box)->rotation.w), 0.999f);
}

TEST(ReferencePhysics, ThreeBoxStackRetainsHeight) {
  World world;
  ground(world);
  const auto a = world.createEntity(), b = world.createEntity(), c = world.createEntity();
  boxBody(world, a, 0.5f);
  boxBody(world, b, 1.5f);
  boxBody(world, c, 2.5f);
  PhysicsSettings settings;
  settings.velocityIterations = 20;
  PhysicsWorld physics(settings);
  for (int i = 0; i < 360; ++i) physics.stepFixed(world);
  EXPECT_NEAR(world.transform(a)->position.y, 0.5f, 0.03f);
  EXPECT_NEAR(world.transform(b)->position.y, 1.5f, 0.04f);
  EXPECT_NEAR(world.transform(c)->position.y, 2.5f, 0.06f);
}

TEST(ReferencePhysics, DampingHasTheSameRateAtDifferentTimesteps) {
  RigidBodyComponent a, b;
  TransformComponent ta, tb;
  a.useGravity = b.useGravity = false;
  a.velocity = b.velocity = {10.0f, 0, 0};
  a.angularVelocity = b.angularVelocity = {0, 2.0f, 0};
  a.linearDamping = b.linearDamping = 0.7f;
  a.angularDamping = b.angularDamping = 0.4f;
  for (int i = 0; i < 120; ++i) integrateRigidBody(a, ta, glm::vec3(0), 1.0f / 120);
  for (int i = 0; i < 60; ++i) integrateRigidBody(b, tb, glm::vec3(0), 1.0f / 60);
  EXPECT_NEAR(a.velocity.x, b.velocity.x, 1e-4f);
  EXPECT_NEAR(a.velocity.x, 10.0f * std::exp(-0.7f), 1e-4f);
  EXPECT_NEAR(a.angularVelocity.y, b.angularVelocity.y, 1e-4f);
}

TEST(ReferencePhysics, ContactFaceStaysNearTheActualOverlap) {
  TransformComponent a, b;
  ColliderComponent ca, cb;
  ca.shape = cb.shape = ColliderShape::Box;
  ca.halfExtents = {30.0f, 0.5f, 30.0f};
  a.position.y = -0.5f;
  b.position = {7.0f, 0.49f, -4.0f};
  const auto contact = collideBoxBox(a, ca, b, cb);
  ASSERT_TRUE(contact);
  ASSERT_EQ(contact->pointCount, 4u);
  EXPECT_NEAR(contact->point.x, 7.0f, 1e-5f);
  EXPECT_NEAR(contact->point.z, -4.0f, 1e-5f);
}

TEST(InputSafety, BroadphaseRejectsInvalidBoundsAndIndices) {
  SpatialHash hash;
  EXPECT_THROW(hash.insert(0, glm::vec3(0), -1.0f), std::invalid_argument);
  EXPECT_THROW(hash.insert(0, glm::vec3(std::numeric_limits<float>::infinity()), 1.0f),
               std::invalid_argument);
  EXPECT_THROW(hash.insertAabb(0, glm::vec3(-10000), glm::vec3(10000)), std::length_error);
  hash.insert(9, glm::vec3(0), 0.5f);
  EXPECT_THROW(hash.queryPairs({0}, {glm::vec3(0)}, {0.5f}), std::out_of_range);
}

TEST(InputSafety, StaleHandleCannotReferToAnEntityAfterClear) {
  World world;
  const auto old = world.createEntity();
  world.clear();
  const auto fresh = world.createEntity();
  EXPECT_FALSE(world.alive(old));
  EXPECT_NE(old.id, fresh.id);
}

TEST(InputSafety, InvalidMassAndColliderDimensionsDoNotProduceNanInertia) {
  RigidBodyComponent body;
  EXPECT_THROW(body.computeInertia(ColliderShape::Sphere, 0.0f, glm::vec3(1)),
               std::invalid_argument);
  EXPECT_THROW(body.computeInertia(ColliderShape::Box, 0.0f, glm::vec3(-1)), std::invalid_argument);
}

TEST(InputSafety, InteriorRayReturnsAFiniteExitNormal) {
  Ray ray;
  ray.direction = {1, 0, 0};
  float distance;
  glm::vec3 normal;
  ASSERT_TRUE(intersectRaySphere(ray, glm::vec3(0), 1.0f, distance, normal));
  EXPECT_FLOAT_EQ(distance, 1.0f);
  EXPECT_FLOAT_EQ(normal.x, 1.0f);
  ASSERT_TRUE(
      intersectRayOBB(ray, glm::vec3(0), glm::quat(1, 0, 0, 0), glm::vec3(1), distance, normal));
  EXPECT_FLOAT_EQ(distance, 1.0f);
  EXPECT_FLOAT_EQ(normal.x, 1.0f);
  ray.direction = glm::vec3(0);
  EXPECT_FALSE(intersectRaySphere(ray, glm::vec3(0), 1.0f, distance, normal));
  EXPECT_FALSE(
      intersectRayOBB(ray, glm::vec3(0), glm::quat(1, 0, 0, 0), glm::vec3(1), distance, normal));
}

TEST(InputSafety, InvalidJointIsRejectedBeforeChangingWorld) {
  World world;
  const auto body = world.createEntity();
  PhysicsWorld physics;
  DistanceJoint joint;
  joint.entityA = body.id;
  joint.targetDistance = std::numeric_limits<float>::quiet_NaN();
  physics.addDistanceJoint(joint);
  EXPECT_THROW(physics.stepFixed(world), std::invalid_argument);
  EXPECT_FLOAT_EQ(world.transform(body)->position.y, 0.0f);
}

TEST(InputSafety, ExcessiveContactCountCannotIndexPastSolverStorage) {
  RigidBodyComponent a, b;
  TransformComponent ta, tb;
  ContactManifold contact;
  contact.pointCount = contact.points.size() + 1;
  ContactSolverState state;
  EXPECT_THROW(solveContactIteration(a, ta, b, tb, contact, state, true, true),
               std::invalid_argument);
}
