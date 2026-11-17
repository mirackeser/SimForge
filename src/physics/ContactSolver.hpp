#pragma once

#include <array>

#include "physics/Collision.hpp"

namespace simforge {

struct ContactImpulse {
  float normal{0.0f};
  glm::vec2 tangent{0.0f};
  float bounceVelocity{0.0f};
};

struct ContactSolverState {
  std::array<ContactImpulse, 8> impulses{};
  bool initialized{false};
};

// Impulses are accumulated across solver iterations within one fixed tick.
void solveContactIteration(RigidBodyComponent& a, const TransformComponent& ta,
                           RigidBodyComponent& b, const TransformComponent& tb,
                           const ContactManifold& contact, ContactSolverState& state, bool friction,
                           bool rotation);

}  // namespace simforge
