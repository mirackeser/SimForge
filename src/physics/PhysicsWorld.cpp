#include "physics/PhysicsWorld.hpp"

#include "physics/Collision.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace simforge {

PhysicsWorld::PhysicsWorld(PhysicsSettings settings)
    : settings_(settings), spatialHash_(settings.spatialCellSize) {}

void PhysicsWorld::resetAccumulatedTime() { accumulator_ = 0.0f; }

void PhysicsWorld::validateSettings() const {
  if (!std::isfinite(settings_.fixedTimestep) || settings_.fixedTimestep <= 0.0f ||
      !std::isfinite(settings_.spatialCellSize) || settings_.spatialCellSize <= 0.0f ||
      settings_.velocityIterations < 1 || settings_.positionIterations < 0 ||
      !std::isfinite(settings_.gravity.x) || !std::isfinite(settings_.gravity.y) ||
      !std::isfinite(settings_.gravity.z)) {
    throw std::invalid_argument("Invalid physics settings");
  }
}

void PhysicsWorld::reset() {
  resetAccumulatedTime();
  simulationTime_ = 0.0;
  collisionCountLastFrame_ = 0;
  profileStats_ = {};
  debugContacts_.clear();
  spatialHash_.clear();
  clearJoints();
}

void PhysicsWorld::step(World& world, float dt) {
  if (!std::isfinite(dt) || dt < 0.0f) {
    throw std::invalid_argument("Frame time must be finite and nonnegative");
  }
  validateSettings();
  accumulator_ += static_cast<double>(dt);
  const double step = settings_.fixedTimestep;
  // Limit catch-up work and discard whole excess ticks, keeping the fraction.
  // A stalled render frame must not leave a growing backlog on later frames.
  if (accumulator_ >= 9.0 * step) {
    accumulator_ = 8.0 * step + std::fmod(accumulator_, step);
  }
  for (int substep = 0; accumulator_ >= step && substep < 8; ++substep) {
    advance(world, settings_.fixedTimestep);
    accumulator_ -= step;
  }
}

void PhysicsWorld::stepFixed(World& world) {
  validateSettings();
  advance(world, settings_.fixedTimestep);
}

void PhysicsWorld::advance(World& world, float dt) {
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  profileStats_ = {};
  if (spatialHash_.cellSize() != settings_.spatialCellSize) {
    spatialHash_ = SpatialHash(settings_.spatialCellSize);
  }

  const auto integrationStart = Clock::now();
  constraintSolver_.applySpringForces(world, springJoints_, mouseSpring_, dt,
                                      settings_.enableRotationalDynamics);
  integrate(world, dt);
  profileStats_.integrationUs = std::chrono::duration<float, std::micro>(Clock::now() - integrationStart).count();

  for (int iter = 0; iter < settings_.velocityIterations; ++iter) {
    const auto constraintStart = Clock::now();
    constraintSolver_.solveVelocityConstraints(world, distanceJoints_, dt, settings_.enableRotationalDynamics);
    profileStats_.constraintSolverUs += std::chrono::duration<float, std::micro>(Clock::now() - constraintStart).count();
    detectAndResolve(world, dt);
  }
  const auto positionStart = Clock::now();
  for (int iter = 0; iter < settings_.positionIterations; ++iter) {
    constraintSolver_.solvePositionConstraints(world, distanceJoints_);
  }
  profileStats_.constraintSolverUs += std::chrono::duration<float, std::micro>(Clock::now() - positionStart).count();
  profileStats_.totalSubstepUs = std::chrono::duration<float, std::micro>(Clock::now() - start).count();
  simulationTime_ += dt;
}

void PhysicsWorld::integrate(World& world, float dt) {
  for (auto& rec : world.entitiesMutable()) {
    if (!rec.active) continue;
    integrateRigidBody(rec.rigidBody, rec.transform, settings_.gravity, dt, settings_.enableRotationalDynamics);
  }
}

void PhysicsWorld::detectAndResolve(World& world, float dt) {
  debugContacts_.clear();
  collisionCountLastFrame_ = 0;
  auto& records = world.entitiesMutable();

  std::vector<EntityId> ids;
  std::vector<glm::vec3> positions;
  std::vector<float> radii;
  std::vector<glm::vec3> bounds;
  ids.reserve(records.size());
  positions.reserve(records.size());
  radii.reserve(records.size());
  bounds.reserve(records.size());

  for (std::size_t i = 0; i < records.size(); ++i) {
    const auto& rec = records[i];
    if (!rec.active) continue;
    ids.push_back(static_cast<EntityId>(i));
    positions.push_back(rec.transform.position);
    float r = (rec.collider.shape == ColliderShape::Sphere)
                  ? rec.collider.radius
                  : glm::length(rec.collider.halfExtents);
    radii.push_back(r);
    glm::vec3 extent(r);
    if (rec.collider.shape == ColliderShape::Box) {
      const glm::mat3 rotation = glm::mat3_cast(rec.transform.rotation);
      const auto& half = rec.collider.halfExtents;
      extent = glm::abs(rotation[0]) * half.x + glm::abs(rotation[1]) * half.y +
               glm::abs(rotation[2]) * half.z;
    }
    bounds.push_back(extent);
  }

  // Broadphase: Spatial Hash
  auto tBroadStart = std::chrono::high_resolution_clock::now();
  spatialHash_.clear();
  for (std::size_t i = 0; i < ids.size(); ++i) {
    spatialHash_.insertAabb(static_cast<EntityId>(i), positions[i] - bounds[i], positions[i] + bounds[i]);
  }
  auto pairs = spatialHash_.queryPairs(ids, positions, radii);
  auto tBroadEnd = std::chrono::high_resolution_clock::now();
  profileStats_.broadphaseUs +=
      std::chrono::duration<float, std::micro>(tBroadEnd - tBroadStart).count();

  // Narrowphase & Contact Resolution
  auto tNarrowStart = std::chrono::high_resolution_clock::now();

  for (const CollisionPair& pair : pairs) {
    if (pair.a >= ids.size() || pair.b >= ids.size()) continue;

    std::size_t indexA = ids[pair.a];
    std::size_t indexB = ids[pair.b];
    if (indexA >= records.size() || indexB >= records.size()) continue;

    auto& recA = records[indexA];
    auto& recB = records[indexB];
    if (!recA.active || !recB.active) continue;
    if (recA.rigidBody.isStatic && recB.rigidBody.isStatic) continue;

    std::optional<ContactManifold> contact;
    const auto& ta = recA.transform;
    const auto& tb = recB.transform;
    const auto& ca = recA.collider;
    const auto& cb = recB.collider;

    if (ca.shape == ColliderShape::Sphere && cb.shape == ColliderShape::Sphere) {
      contact = collideSphereSphere(ta, ca, tb, cb);
    } else if (ca.shape == ColliderShape::Sphere && cb.shape == ColliderShape::Box) {
      contact = collideSphereBox(ta, ca, tb, cb);
    } else if (ca.shape == ColliderShape::Box && cb.shape == ColliderShape::Sphere) {
      contact = collideSphereBox(tb, cb, ta, ca);
      if (contact) contact->normal *= -1.0f;
    } else {
      contact = collideBoxBox(ta, ca, tb, cb);
    }

    if (!contact) continue;

    ++collisionCountLastFrame_;
    if (debugContacts_.size() < 400) {
      debugContacts_.push_back(*contact);
    }

    // Resolve velocity impulse (Normal + Coulomb Friction + Rotational Torque)
    resolveContact(recA.rigidBody, recA.transform, recB.rigidBody, recB.transform, *contact,
                   settings_.enableFriction, settings_.enableRotationalDynamics);

    // Position correction to prevent sinking/penetration
    const float slop = 0.003f;
    const float percent = 0.65f;
    float invMassSum = (recA.rigidBody.isStatic ? 0.0f : recA.rigidBody.inverseMass) +
                       (recB.rigidBody.isStatic ? 0.0f : recB.rigidBody.inverseMass);
    if (invMassSum > 0.0f) {
      glm::vec3 correction =
          contact->normal * (std::max(contact->penetration - slop, 0.0f) * percent / invMassSum);
      if (!recA.rigidBody.isStatic) recA.transform.position -= correction * recA.rigidBody.inverseMass;
      if (!recB.rigidBody.isStatic) recB.transform.position += correction * recB.rigidBody.inverseMass;
    }
  }

  auto tNarrowEnd = std::chrono::high_resolution_clock::now();
  profileStats_.narrowphaseUs +=
      std::chrono::duration<float, std::micro>(tNarrowEnd - tNarrowStart).count();
}

}  // namespace simforge
