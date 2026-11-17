#include "physics/ContactSolver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "core/Validation.hpp"

namespace simforge {
namespace {

float inverseMass(const RigidBodyComponent& body) {
  return body.isStatic ? 0.0f : body.inverseMass;
}

glm::vec3 pointVelocity(const RigidBodyComponent& body, const glm::vec3& arm, bool rotation) {
  if (body.isStatic) return glm::vec3(0.0f);
  return body.velocity + (rotation ? glm::cross(body.angularVelocity, arm) : glm::vec3(0.0f));
}

float effectiveMass(const RigidBodyComponent& a, const RigidBodyComponent& b, const glm::vec3& ra,
                    const glm::vec3& rb, const glm::vec3& axis, bool rotation) {
  float result = inverseMass(a) + inverseMass(b);
  if (rotation && !a.isStatic)
    result += glm::dot(axis, glm::cross(a.inverseInertiaTensorWorld * glm::cross(ra, axis), ra));
  if (rotation && !b.isStatic)
    result += glm::dot(axis, glm::cross(b.inverseInertiaTensorWorld * glm::cross(rb, axis), rb));
  return result;
}

void applyImpulse(RigidBodyComponent& body, const glm::vec3& arm, const glm::vec3& impulse,
                  bool rotation) {
  if (body.isStatic) return;
  body.velocity += impulse * body.inverseMass;
  if (rotation) body.angularVelocity += body.inverseInertiaTensorWorld * glm::cross(arm, impulse);
}

}  // namespace

void solveContactIteration(RigidBodyComponent& a, const TransformComponent& ta,
                           RigidBodyComponent& b, const TransformComponent& tb,
                           const ContactManifold& contact, ContactSolverState& state, bool friction,
                           bool rotation) {
  if (contact.pointCount > contact.points.size())
    throw std::invalid_argument("Invalid contact point count");
  if (!finite(contact.normal) ||
      std::abs(glm::dot(contact.normal, contact.normal) - 1.0f) > 0.01f || !finite(contact.point))
    throw std::invalid_argument("Invalid contact geometry");
  for (std::size_t i = 0; i < contact.pointCount; ++i) {
    if (!finite(contact.points[i])) throw std::invalid_argument("Invalid contact point");
  }
  if (a.isStatic && b.isStatic) return;
  const glm::vec3 n = contact.normal;
  const glm::vec3 tangentU =
      glm::normalize(glm::cross(n, std::abs(n.x) < 0.6f ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0)));
  const glm::vec3 tangentV = glm::cross(n, tangentU);
  const std::size_t count = contact.pointCount == 0 ? 1 : contact.pointCount;
  const float restitution = std::min(a.restitution, b.restitution);
  const float muStatic = std::sqrt(a.staticFriction * b.staticFriction);
  const float muDynamic = std::sqrt(a.dynamicFriction * b.dynamicFriction);

  // Restitution is based on the incoming velocity, before any point in the manifold is solved.
  if (!state.initialized) {
    for (std::size_t i = 0; i < count; ++i) {
      const auto point = contact.pointCount == 0 ? contact.point : contact.points[i];
      const auto velocity = pointVelocity(b, point - tb.position, rotation) -
                            pointVelocity(a, point - ta.position, rotation);
      const float speed = glm::dot(velocity, n);
      state.impulses[i].bounceVelocity = speed < -1.0f ? -restitution * speed : 0.0f;
    }
    state.initialized = true;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const auto point = contact.pointCount == 0 ? contact.point : contact.points[i];
    const glm::vec3 ra = point - ta.position, rb = point - tb.position;
    const float massN = effectiveMass(a, b, ra, rb, n, rotation);
    if (massN <= 1e-8f) continue;
    auto& accumulated = state.impulses[i];
    auto velocity = pointVelocity(b, rb, rotation) - pointVelocity(a, ra, rotation);
    const float previousNormal = accumulated.normal;
    accumulated.normal = std::max(
        0.0f, previousNormal + (accumulated.bounceVelocity - glm::dot(velocity, n)) / massN);
    const glm::vec3 impulse = (accumulated.normal - previousNormal) * n;
    applyImpulse(a, ra, -impulse, rotation);
    applyImpulse(b, rb, impulse, rotation);
    if (!friction) continue;

    velocity = pointVelocity(b, rb, rotation) - pointVelocity(a, ra, rotation);
    const float massU = effectiveMass(a, b, ra, rb, tangentU, rotation);
    const float massV = effectiveMass(a, b, ra, rb, tangentV, rotation);
    if (massU <= 1e-8f || massV <= 1e-8f) continue;
    glm::vec2 next = accumulated.tangent - glm::vec2(glm::dot(velocity, tangentU) / massU,
                                                     glm::dot(velocity, tangentV) / massV);
    const float length = glm::length(next);
    if (length > muStatic * accumulated.normal && length > 1e-8f) {
      next *= (muDynamic * accumulated.normal) / length;
    }
    const glm::vec2 delta = next - accumulated.tangent;
    accumulated.tangent = next;
    const glm::vec3 tangentImpulse = tangentU * delta.x + tangentV * delta.y;
    applyImpulse(a, ra, -tangentImpulse, rotation);
    applyImpulse(b, rb, tangentImpulse, rotation);
  }
}

}  // namespace simforge
