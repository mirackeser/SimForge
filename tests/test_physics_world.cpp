#include "physics/PhysicsWorld.hpp"
#include "scene/DemoScenes.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace simforge;

namespace {
Entity movingSphere(World& world) {
  const Entity entity = world.createEntity("moving");
  auto* body = world.rigidBody(entity);
  body->useGravity = false;
  body->linearDamping = 0.0f;
  body->angularDamping = 0.0f;
  body->velocity = {1.0f, 0.0f, 0.0f};
  body->computeInertia(ColliderShape::Sphere, 0.5f, glm::vec3(0.0f));
  return entity;
}
}

TEST(PhysicsWorld, FramePartitionsProduceSameState) {
  World a, b;
  const auto ea = movingSphere(a), eb = movingSphere(b);
  PhysicsWorld pa, pb;
  for (int i = 0; i < 120; ++i) pa.step(a, pa.settings().fixedTimestep);
  for (int i = 0; i < 60; ++i) pb.step(b, 2.0f * pb.settings().fixedTimestep);
  EXPECT_NEAR(a.transform(ea)->position.x, b.transform(eb)->position.x, 1e-6f);
  EXPECT_NEAR(pa.simulationTime(), pb.simulationTime(), 1e-6f);
}

TEST(PhysicsWorld, SphereOnGroundIsPushedOutAndStopsFalling) {
  // Check both pair orders: the normal contract is A -> B for every shape combination.
  for (bool groundFirst : {false, true}) {
    World world;
    const auto first = world.createEntity();
    const auto second = world.createEntity();
    const auto ground = groundFirst ? first : second;
    const auto sphere = groundFirst ? second : first;
    auto* groundBody = world.rigidBody(ground);
    groundBody->isStatic = true;
    groundBody->inverseMass = 0.0f;
    auto* collider = world.collider(ground);
    collider->shape = ColliderShape::Box;
    collider->halfExtents = {2.0f, 0.5f, 2.0f};
    world.transform(ground)->position.y = -0.5f;
    world.transform(sphere)->position.y = 0.45f;
    auto* body = world.rigidBody(sphere);
    body->velocity.y = -1.0f;
    body->linearDamping = 0.0f;
    body->computeInertia(ColliderShape::Sphere, 0.5f, glm::vec3(0.0f));
    PhysicsWorld physics;
    physics.step(world, physics.settings().fixedTimestep);
    EXPECT_GT(world.transform(sphere)->position.y, 0.45f);
    EXPECT_GE(body->velocity.y, -1e-4f);
  }
}

TEST(PhysicsWorld, DiscardsOverloadBacklog) {
  World world;
  const auto entity = movingSphere(world);
  PhysicsWorld physics;
  physics.step(world, 1.0f);
  EXPECT_LE(physics.simulationTime(), 8.0f * physics.settings().fixedTimestep + 1e-6f);
  const float position = world.transform(entity)->position.x;
  physics.step(world, 0.0f);
  EXPECT_FLOAT_EQ(world.transform(entity)->position.x, position);
}

TEST(PhysicsWorld, RejectsInvalidFrameTimeBeforeMutatingWorld) {
  World world;
  const auto entity = movingSphere(world);
  PhysicsWorld physics;
  for (float dt : {-1.0f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
    EXPECT_THROW(physics.step(world, dt), std::invalid_argument);
  }
  EXPECT_FLOAT_EQ(world.transform(entity)->position.x, 0.0f);
  physics.step(world, physics.settings().fixedTimestep);
  EXPECT_GT(world.transform(entity)->position.x, 0.0f);
}

TEST(PhysicsWorld, ReloadingSceneResetsClockAndContacts) {
  World world;
  PhysicsWorld physics;
  loadDemoScene(world, physics, DemoScene::NewtonsCradle);
  physics.step(world, physics.settings().fixedTimestep);
  ASSERT_GT(physics.simulationTime(), 0.0f);
  loadDemoScene(world, physics, DemoScene::NewtonsCradle);
  EXPECT_FLOAT_EQ(physics.simulationTime(), 0.0f);
  EXPECT_TRUE(physics.debugContacts().empty());
}

TEST(PhysicsWorld, SpringForceDoesNotScaleWithVelocityIterations) {
  float speeds[2]{};
  for (int i = 0; i < 2; ++i) {
    World world;
    const auto anchor = world.createEntity();
    const auto bob = movingSphere(world);
    world.rigidBody(anchor)->isStatic = true;
    world.rigidBody(anchor)->inverseMass = 0.0f;
    world.transform(bob)->position.x = 3.0f;
    world.rigidBody(bob)->velocity = glm::vec3(0.0f);
    PhysicsSettings settings;
    settings.velocityIterations = i == 0 ? 1 : 8;
    PhysicsWorld physics(settings);
    SpringJoint spring;
    spring.entityA = anchor.id;
    spring.entityB = bob.id;
    spring.restLength = 2.0f;
    spring.stiffness = 12.0f;
    spring.damping = 0.0f;
    physics.addSpringJoint(spring);
    physics.step(world, settings.fixedTimestep);
    speeds[i] = world.rigidBody(bob)->velocity.x;
  }
  EXPECT_LT(speeds[0], 0.0f);
  EXPECT_NEAR(speeds[0], speeds[1], 1e-6f);
}

TEST(PhysicsWorld, FixedStepIgnoresRenderRemainder) {
  World world;
  const auto entity = movingSphere(world);
  PhysicsWorld physics;
  const float dt = physics.settings().fixedTimestep;
  physics.step(world, 0.5f * dt);
  physics.stepFixed(world);
  EXPECT_NEAR(world.transform(entity)->position.x, dt, 1e-7f);
  physics.step(world, 0.5f * dt);
  EXPECT_NEAR(world.transform(entity)->position.x, 2.0f * dt, 1e-7f);
}

TEST(PhysicsWorld, InvalidSettingsCanBeCorrectedWithoutPoisoningAccumulator) {
  World world;
  PhysicsWorld physics;
  const float dt = physics.settings().fixedTimestep;
  for (float value : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN()}) {
    physics.settings().fixedTimestep = value;
    EXPECT_THROW(physics.step(world, dt), std::invalid_argument);
  }
  physics.settings().fixedTimestep = dt;
  physics.settings().spatialCellSize = 0.0f;
  EXPECT_THROW(physics.stepFixed(world), std::invalid_argument);
  physics.settings().spatialCellSize = 1.0f;
  physics.stepFixed(world);
  EXPECT_DOUBLE_EQ(physics.simulationTime(), static_cast<double>(dt));
  EXPECT_FLOAT_EQ(physics.spatialHash().cellSize(), 1.0f);
}

TEST(PhysicsWorld, RotationTogglePreservesOrientationAndSpin) {
  World world;
  const auto entity = movingSphere(world);
  auto* body = world.rigidBody(entity);
  body->angularVelocity = {0.0f, 2.0f, 0.0f};
  PhysicsSettings settings;
  settings.enableRotationalDynamics = false;
  PhysicsWorld physics(settings);
  physics.stepFixed(world);
  EXPECT_FLOAT_EQ(world.transform(entity)->rotation.w, 1.0f);
  EXPECT_FLOAT_EQ(body->angularVelocity.y, 2.0f);
  physics.settings().enableRotationalDynamics = true;
  physics.stepFixed(world);
  EXPECT_GT(world.transform(entity)->rotation.y, 0.0f);
}

TEST(PhysicsWorld, FrictionToggleControlsTangentialImpulse) {
  float speeds[2]{};
  for (int i = 0; i < 2; ++i) {
    World world;
    const auto ground = world.createEntity();
    const auto sphere = movingSphere(world);
    auto* g = world.rigidBody(ground);
    g->isStatic = true;  // Effective inverse mass is zero even with a stale stored value.
    auto* c = world.collider(ground);
    c->shape = ColliderShape::Box;
    c->halfExtents = {2.0f, 0.5f, 2.0f};
    world.transform(ground)->position.y = -0.5f;
    world.transform(sphere)->position.y = 0.45f;
    world.rigidBody(sphere)->velocity = {2.0f, -1.0f, 0.0f};
    PhysicsSettings settings;
    settings.enableFriction = i == 1;
    PhysicsWorld physics(settings);
    physics.stepFixed(world);
    speeds[i] = world.rigidBody(sphere)->velocity.x;
  }
  EXPECT_FLOAT_EQ(speeds[0], 2.0f);
  EXPECT_LT(speeds[1], speeds[0]);
}

TEST(PhysicsWorld, BroadphaseKeepsContactNearRotatedThinBoxTip) {
  World world;
  const auto box = world.createEntity();
  const auto sphere = movingSphere(world);
  world.rigidBody(box)->isStatic = true;
  world.rigidBody(box)->inverseMass = 0.0f;
  world.collider(box)->shape = ColliderShape::Box;
  world.collider(box)->halfExtents = {3.0f, 0.1f, 0.1f};
  const auto rotation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));
  world.transform(box)->rotation = rotation;
  const glm::vec3 direction = rotation * glm::vec3(1.0f, 0.0f, 0.0f);
  world.transform(sphere)->position = direction * 3.2f;
  world.rigidBody(sphere)->velocity = -direction;
  PhysicsSettings settings;
  settings.velocityIterations = 1;
  settings.spatialCellSize = 0.5f;
  PhysicsWorld physics(settings);
  physics.stepFixed(world);
  EXPECT_EQ(physics.collisionCountLastFrame(), 1);
  EXPECT_GT(glm::dot(world.rigidBody(sphere)->velocity, direction), -0.01f);
}
