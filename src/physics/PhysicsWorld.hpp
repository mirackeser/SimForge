#pragma once

#include <chrono>
#include <vector>

#include "ecs/World.hpp"
#include "physics/Collision.hpp"
#include "physics/Constraint.hpp"
#include "physics/ContactSolver.hpp"
#include "physics/SpatialHash.hpp"

namespace simforge {

struct PhysicsSettings {
  glm::vec3 gravity{0.0f, -9.81f, 0.0f};
  float fixedTimestep{1.0f / 120.0f};
  int velocityIterations{8};
  int positionIterations{3};
  float spatialCellSize{2.0f};
  bool enableRotationalDynamics{true};
  bool enableFriction{true};
};

struct PhysicsProfileStats {
  float broadphaseUs{0.0f};
  float narrowphaseUs{0.0f};
  float constraintSolverUs{0.0f};
  float integrationUs{0.0f};
  float totalSubstepUs{0.0f};
};

class PhysicsWorld {
 public:
  explicit PhysicsWorld(PhysicsSettings settings = {});

  void step(World& world, float dt);
  // Advance exactly one fixed tick, independent of the render-time accumulator.
  void stepFixed(World& world);
  // Clear all scene-specific state, including joints and simulation clock.
  void reset();
  void resetAccumulatedTime();

  PhysicsSettings& settings() { return settings_; }
  const PhysicsSettings& settings() const { return settings_; }

  int collisionCountLastFrame() const { return collisionCountLastFrame_; }
  double simulationTime() const { return simulationTime_; }
  const PhysicsProfileStats& profileStats() const { return profileStats_; }

  // Constraints & Joints
  void addDistanceJoint(const DistanceJoint& joint) { distanceJoints_.push_back(joint); }
  void addSpringJoint(const SpringJoint& joint) { springJoints_.push_back(joint); }
  void clearJoints() {
    distanceJoints_.clear();
    springJoints_.clear();
    mouseSpring_.active = false;
  }

  const std::vector<DistanceJoint>& distanceJoints() const { return distanceJoints_; }
  const std::vector<SpringJoint>& springJoints() const { return springJoints_; }

  // Interactive mouse spring
  void setMouseSpring(const MouseSpring& ms) { mouseSpring_ = ms; }
  MouseSpring& mouseSpring() { return mouseSpring_; }
  const MouseSpring& mouseSpring() const { return mouseSpring_; }

  // Debug contacts and spatial hash
  const std::vector<ContactManifold>& debugContacts() const { return debugContacts_; }
  const SpatialHash& spatialHash() const { return spatialHash_; }

 private:
  void validateSettings() const;
  void validateWorld(const World& world) const;
  void advance(World& world, float dt);
  void integrate(World& world, float dt);
  void detectContacts(World& world);
  void correctContactPositions(World& world);
  struct SolverContact {
    std::size_t indexA;
    std::size_t indexB;
    ContactManifold manifold;
    ContactSolverState state;
  };
  std::vector<SolverContact> solverContacts_;

  PhysicsSettings settings_;
  SpatialHash spatialHash_;
  ConstraintSolver constraintSolver_;

  std::vector<DistanceJoint> distanceJoints_;
  std::vector<SpringJoint> springJoints_;
  MouseSpring mouseSpring_;

  std::vector<ContactManifold> debugContacts_;

  double accumulator_{0.0};
  double simulationTime_{0.0};
  int collisionCountLastFrame_{0};
  PhysicsProfileStats profileStats_;
};

}  // namespace simforge
