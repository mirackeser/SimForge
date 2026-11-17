#pragma once

#include <glm/glm.hpp>
#include <vector>

#include "core/Types.hpp"
#include "ecs/World.hpp"

namespace simforge {

struct DistanceJoint {
  EntityId entityA{kInvalidEntity};
  EntityId entityB{kInvalidEntity};
  glm::vec3 localAnchorA{0.0f};
  glm::vec3 localAnchorB{0.0f};
  float targetDistance{1.0f};
  float stiffness{1.0f};  // 1.0 = rigid distance constraint
};

struct SpringJoint {
  EntityId entityA{kInvalidEntity};
  EntityId entityB{kInvalidEntity};
  glm::vec3 localAnchorA{0.0f};
  glm::vec3 localAnchorB{0.0f};
  float restLength{1.0f};
  float stiffness{120.0f};
  float damping{6.0f};
};

struct MouseSpring {
  EntityId entity{kInvalidEntity};
  glm::vec3 localAnchor{0.0f};
  glm::vec3 worldTarget{0.0f};
  float stiffness{250.0f};
  float damping{15.0f};
  bool active{false};
};

class ConstraintSolver {
 public:
  // Apply spring forces once per fixed substep, before integration.
  void applySpringForces(World& world, const std::vector<SpringJoint>& springJoints,
                         const MouseSpring& mouseSpring, float dt, bool enableRotation = true);

  void solveVelocityConstraints(World& world, const std::vector<DistanceJoint>& distanceJoints,
                                float dt, bool enableRotation = true);

  void solvePositionConstraints(World& world, const std::vector<DistanceJoint>& distanceJoints);
};

}  // namespace simforge
