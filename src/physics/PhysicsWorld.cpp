#include "physics/PhysicsWorld.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "core/Validation.hpp"
#include "physics/Collision.hpp"

namespace simforge {

PhysicsWorld::PhysicsWorld(PhysicsSettings settings)
    : settings_(settings), spatialHash_(settings.spatialCellSize) {}

void PhysicsWorld::resetAccumulatedTime() { accumulator_ = 0.0f; }

void PhysicsWorld::validateSettings() const {
  if (!std::isfinite(settings_.fixedTimestep) || settings_.fixedTimestep < 1e-5f ||
      settings_.fixedTimestep > 0.1f || !std::isfinite(settings_.spatialCellSize) ||
      settings_.spatialCellSize < 0.05f || settings_.spatialCellSize > 100.0f ||
      settings_.velocityIterations < 1 || settings_.velocityIterations > 64 ||
      settings_.positionIterations < 0 || settings_.positionIterations > 32 ||
      !std::isfinite(settings_.gravity.x) || !std::isfinite(settings_.gravity.y) ||
      !std::isfinite(settings_.gravity.z)) {
    throw std::invalid_argument("Invalid physics settings");
  }
}

void PhysicsWorld::validateWorld(const World& world) const {
  for (const auto& record : world.entities()) {
    if (!record.active) continue;
    const auto& transform = record.transform;
    const auto& body = record.rigidBody;
    const auto& collider = record.collider;
    if (!finite(transform.position) ||
        glm::any(glm::greaterThan(glm::abs(transform.position), glm::vec3(1000000.0f))) ||
        !finite(transform.rotation) ||
        std::abs(glm::dot(transform.rotation, transform.rotation) - 1.0f) > 0.01f ||
        !finite(body.velocity) || !finite(body.angularVelocity) ||
        !std::isfinite(body.inverseMass) || body.inverseMass < 0.0f ||
        !std::isfinite(body.restitution) || body.restitution < 0.0f || body.restitution > 1.0f ||
        !std::isfinite(body.staticFriction) || body.staticFriction < 0.0f ||
        !std::isfinite(body.dynamicFriction) || body.dynamicFriction < 0.0f ||
        body.dynamicFriction > body.staticFriction || !std::isfinite(body.linearDamping) ||
        body.linearDamping < 0.0f || !std::isfinite(body.angularDamping) ||
        body.angularDamping < 0.0f) {
      throw std::invalid_argument("Invalid rigid-body state");
    }
    for (int axis = 0; axis < 3; ++axis) {
      if (!finite(body.inverseInertiaTensorBody[axis]) ||
          !finite(body.inverseInertiaTensorWorld[axis])) {
        throw std::invalid_argument("Invalid inertia tensor");
      }
    }
    if ((collider.shape == ColliderShape::Sphere &&
         (!std::isfinite(collider.radius) || collider.radius <= 0.0f)) ||
        (collider.shape == ColliderShape::Box &&
         (!finite(collider.halfExtents) ||
          glm::any(glm::lessThanEqual(collider.halfExtents, glm::vec3(0.0f))))) ||
        (collider.shape != ColliderShape::Sphere && collider.shape != ColliderShape::Box)) {
      throw std::invalid_argument("Invalid collider dimensions");
    }
  }
  for (const auto& joint : distanceJoints_) {
    if (!finite(joint.localAnchorA) || !finite(joint.localAnchorB) ||
        !std::isfinite(joint.targetDistance) || joint.targetDistance < 0.0f ||
        !std::isfinite(joint.stiffness) || joint.stiffness < 0.0f || joint.stiffness > 1.0f) {
      throw std::invalid_argument("Invalid distance joint");
    }
  }
  for (const auto& joint : springJoints_) {
    if (!finite(joint.localAnchorA) || !finite(joint.localAnchorB) ||
        !std::isfinite(joint.restLength) || joint.restLength < 0.0f ||
        !std::isfinite(joint.stiffness) || joint.stiffness < 0.0f || joint.stiffness > 1000000.0f ||
        !std::isfinite(joint.damping) || joint.damping < 0.0f || joint.damping > 1000000.0f) {
      throw std::invalid_argument("Invalid spring joint");
    }
  }
  if (mouseSpring_.active &&
      (!finite(mouseSpring_.localAnchor) || !finite(mouseSpring_.worldTarget) ||
       !std::isfinite(mouseSpring_.stiffness) || mouseSpring_.stiffness < 0.0f ||
       mouseSpring_.stiffness > 1000000.0f || !std::isfinite(mouseSpring_.damping) ||
       mouseSpring_.damping < 0.0f || mouseSpring_.damping > 1000000.0f)) {
    throw std::invalid_argument("Invalid mouse spring");
  }
}

void PhysicsWorld::reset() {
  resetAccumulatedTime();
  simulationTime_ = 0.0;
  collisionCountLastFrame_ = 0;
  profileStats_ = {};
  debugContacts_.clear();
  solverContacts_.clear();
  spatialHash_.clear();
  clearJoints();
}

void PhysicsWorld::step(World& world, float dt) {
  if (!std::isfinite(dt) || dt < 0.0f) {
    throw std::invalid_argument("Frame time must be finite and nonnegative");
  }
  validateSettings();
  validateWorld(world);
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
  validateWorld(world);
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
  profileStats_.integrationUs =
      std::chrono::duration<float, std::micro>(Clock::now() - integrationStart).count();

  detectContacts(world);
  auto& records = world.entitiesMutable();
  for (int iter = 0; iter < settings_.velocityIterations; ++iter) {
    const auto constraintStart = Clock::now();
    constraintSolver_.solveVelocityConstraints(world, distanceJoints_, dt,
                                               settings_.enableRotationalDynamics);
    profileStats_.constraintSolverUs +=
        std::chrono::duration<float, std::micro>(Clock::now() - constraintStart).count();
    const auto contactStart = Clock::now();
    for (auto& contact : solverContacts_) {
      auto& a = records[contact.indexA];
      auto& b = records[contact.indexB];
      solveContactIteration(a.rigidBody, a.transform, b.rigidBody, b.transform, contact.manifold,
                            contact.state, settings_.enableFriction,
                            settings_.enableRotationalDynamics);
    }
    profileStats_.narrowphaseUs +=
        std::chrono::duration<float, std::micro>(Clock::now() - contactStart).count();
  }
  correctContactPositions(world);
  const auto positionStart = Clock::now();
  for (int iter = 0; iter < settings_.positionIterations; ++iter) {
    constraintSolver_.solvePositionConstraints(world, distanceJoints_);
  }
  profileStats_.constraintSolverUs +=
      std::chrono::duration<float, std::micro>(Clock::now() - positionStart).count();
  profileStats_.totalSubstepUs =
      std::chrono::duration<float, std::micro>(Clock::now() - start).count();
  simulationTime_ += dt;
}

void PhysicsWorld::integrate(World& world, float dt) {
  for (auto& rec : world.entitiesMutable()) {
    if (!rec.active) continue;
    integrateRigidBody(rec.rigidBody, rec.transform, settings_.gravity, dt,
                       settings_.enableRotationalDynamics);
  }
}

void PhysicsWorld::detectContacts(World& world) {
  debugContacts_.clear();
  solverContacts_.clear();
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
    float r = (rec.collider.shape == ColliderShape::Sphere) ? rec.collider.radius
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
    spatialHash_.insertAabb(static_cast<EntityId>(i), positions[i] - bounds[i],
                            positions[i] + bounds[i]);
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

    solverContacts_.push_back({indexA, indexB, *contact, {}});
  }

  auto tNarrowEnd = std::chrono::high_resolution_clock::now();
  profileStats_.narrowphaseUs +=
      std::chrono::duration<float, std::micro>(tNarrowEnd - tNarrowStart).count();
}

void PhysicsWorld::correctContactPositions(World& world) {
  auto& records = world.entitiesMutable();
  for (const auto& contact : solverContacts_) {
    auto& a = records[contact.indexA];
    auto& b = records[contact.indexB];
    const float invA = a.rigidBody.isStatic ? 0.0f : a.rigidBody.inverseMass;
    const float invB = b.rigidBody.isStatic ? 0.0f : b.rigidBody.inverseMass;
    if (invA + invB <= 0.0f) continue;
    const glm::vec3 correction =
        contact.manifold.normal *
        (std::max(contact.manifold.penetration - 0.001f, 0.0f) * 0.75f / (invA + invB));
    a.transform.position -= correction * invA;
    b.transform.position += correction * invB;
  }
}

}  // namespace simforge
