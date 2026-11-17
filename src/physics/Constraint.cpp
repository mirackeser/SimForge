#include "physics/Constraint.hpp"

#include <algorithm>
#include <cmath>
#include <glm/gtc/quaternion.hpp>

namespace simforge {

void ConstraintSolver::solveVelocityConstraints(World& world,
                                                const std::vector<DistanceJoint>& distanceJoints,
                                                float dt, bool enableRotation) {
  // 1. Solve Distance Joints
  for (const auto& joint : distanceJoints) {
    auto* tA = world.transform(Entity{joint.entityA});
    auto* rbA = world.rigidBody(Entity{joint.entityA});
    auto* tB = world.transform(Entity{joint.entityB});
    auto* rbB = world.rigidBody(Entity{joint.entityB});

    if (!tA || !rbA || !tB || !rbB) continue;
    if (rbA->isStatic && rbB->isStatic) continue;

    glm::vec3 rA = tA->rotation * joint.localAnchorA;
    glm::vec3 rB = tB->rotation * joint.localAnchorB;

    glm::vec3 worldA = tA->position + rA;
    glm::vec3 worldB = tB->position + rB;

    glm::vec3 delta = worldB - worldA;
    float currentDist = glm::length(delta);
    if (currentDist < 1e-5f) continue;

    glm::vec3 n = delta / currentDist;

    glm::vec3 vA =
        rbA->velocity + ((rbA->isStatic || !enableRotation) ? glm::vec3(0.0f)
                                                            : glm::cross(rbA->angularVelocity, rA));
    glm::vec3 vB =
        rbB->velocity + ((rbB->isStatic || !enableRotation) ? glm::vec3(0.0f)
                                                            : glm::cross(rbB->angularVelocity, rB));
    float relVel = glm::dot(vB - vA, n);

    // Baumgarte stabilization term for position error
    float posError = currentDist - joint.targetDistance;
    float bias = (0.2f / std::max(dt, 1e-4f)) * posError;

    glm::vec3 angA = (rbA->isStatic || !enableRotation)
                         ? glm::vec3(0.0f)
                         : glm::cross(rbA->inverseInertiaTensorWorld * glm::cross(rA, n), rA);
    glm::vec3 angB = (rbB->isStatic || !enableRotation)
                         ? glm::vec3(0.0f)
                         : glm::cross(rbB->inverseInertiaTensorWorld * glm::cross(rB, n), rB);

    float effectiveMass = (rbA->isStatic ? 0.0f : rbA->inverseMass) +
                          (rbB->isStatic ? 0.0f : rbB->inverseMass) + glm::dot(n, angA + angB);
    if (effectiveMass <= 0.0f) continue;

    float lambda = -(relVel + bias) / effectiveMass * joint.stiffness;
    glm::vec3 impulse = lambda * n;

    if (!rbA->isStatic) {
      rbA->velocity -= impulse * rbA->inverseMass;
      if (enableRotation)
        rbA->angularVelocity -= rbA->inverseInertiaTensorWorld * glm::cross(rA, impulse);
    }
    if (!rbB->isStatic) {
      rbB->velocity += impulse * rbB->inverseMass;
      if (enableRotation)
        rbB->angularVelocity += rbB->inverseInertiaTensorWorld * glm::cross(rB, impulse);
    }
  }
}

void ConstraintSolver::applySpringForces(World& world, const std::vector<SpringJoint>& springJoints,
                                         const MouseSpring& mouseSpring, float dt,
                                         bool enableRotation) {
  // 2. Solve Spring Joints
  for (const auto& spring : springJoints) {
    auto* tA = world.transform(Entity{spring.entityA});
    auto* rbA = world.rigidBody(Entity{spring.entityA});
    auto* tB = world.transform(Entity{spring.entityB});
    auto* rbB = world.rigidBody(Entity{spring.entityB});

    if (!tA || !rbA || !tB || !rbB) continue;

    glm::vec3 rA = tA->rotation * spring.localAnchorA;
    glm::vec3 rB = tB->rotation * spring.localAnchorB;

    glm::vec3 worldA = tA->position + rA;
    glm::vec3 worldB = tB->position + rB;

    glm::vec3 delta = worldB - worldA;
    float currentDist = glm::length(delta);
    if (currentDist < 1e-5f) continue;

    glm::vec3 n = delta / currentDist;
    float displacement = currentDist - spring.restLength;

    glm::vec3 vA =
        rbA->velocity + ((rbA->isStatic || !enableRotation) ? glm::vec3(0.0f)
                                                            : glm::cross(rbA->angularVelocity, rA));
    glm::vec3 vB =
        rbB->velocity + ((rbB->isStatic || !enableRotation) ? glm::vec3(0.0f)
                                                            : glm::cross(rbB->angularVelocity, rB));
    float relVel = glm::dot(vB - vA, n);

    float forceMagnitude = spring.stiffness * displacement + spring.damping * relVel;
    glm::vec3 force = forceMagnitude * n;

    if (!rbA->isStatic) {
      rbA->velocity += force * rbA->inverseMass * dt;
      if (enableRotation)
        rbA->angularVelocity += rbA->inverseInertiaTensorWorld * glm::cross(rA, force) * dt;
    }
    if (!rbB->isStatic) {
      rbB->velocity -= force * rbB->inverseMass * dt;
      if (enableRotation)
        rbB->angularVelocity -= rbB->inverseInertiaTensorWorld * glm::cross(rB, force) * dt;
    }
  }

  // 3. Solve Mouse Spring
  if (mouseSpring.active && mouseSpring.entity != kInvalidEntity) {
    auto* t = world.transform(Entity{mouseSpring.entity});
    auto* rb = world.rigidBody(Entity{mouseSpring.entity});
    if (t && rb && !rb->isStatic) {
      glm::vec3 r = t->rotation * mouseSpring.localAnchor;
      glm::vec3 anchorWorld = t->position + r;
      glm::vec3 delta = mouseSpring.worldTarget - anchorWorld;

      glm::vec3 vAnchor =
          rb->velocity + (enableRotation ? glm::cross(rb->angularVelocity, r) : glm::vec3(0.0f));
      glm::vec3 force = mouseSpring.stiffness * delta - mouseSpring.damping * vAnchor;

      rb->velocity += force * rb->inverseMass * dt;
      if (enableRotation)
        rb->angularVelocity += rb->inverseInertiaTensorWorld * glm::cross(r, force) * dt;
    }
  }
}

void ConstraintSolver::solvePositionConstraints(World& world,
                                                const std::vector<DistanceJoint>& distanceJoints) {
  for (const auto& joint : distanceJoints) {
    auto* tA = world.transform(Entity{joint.entityA});
    auto* rbA = world.rigidBody(Entity{joint.entityA});
    auto* tB = world.transform(Entity{joint.entityB});
    auto* rbB = world.rigidBody(Entity{joint.entityB});

    if (!tA || !rbA || !tB || !rbB) continue;
    if (rbA->isStatic && rbB->isStatic) continue;

    glm::vec3 rA = tA->rotation * joint.localAnchorA;
    glm::vec3 rB = tB->rotation * joint.localAnchorB;

    glm::vec3 worldA = tA->position + rA;
    glm::vec3 worldB = tB->position + rB;

    glm::vec3 delta = worldB - worldA;
    float currentDist = glm::length(delta);
    if (currentDist < 1e-5f) continue;

    float error = currentDist - joint.targetDistance;
    float totalInvMass =
        (rbA->isStatic ? 0.0f : rbA->inverseMass) + (rbB->isStatic ? 0.0f : rbB->inverseMass);
    if (totalInvMass <= 0.0f) continue;

    glm::vec3 correction = (delta / currentDist) * (error / totalInvMass) * 0.75f;
    if (!rbA->isStatic) tA->position += correction * rbA->inverseMass;
    if (!rbB->isStatic) tB->position -= correction * rbB->inverseMass;
  }
}

}  // namespace simforge
