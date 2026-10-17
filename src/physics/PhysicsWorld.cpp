#include "physics/PhysicsWorld.hpp"

#include "physics/Collision.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace simforge {

PhysicsWorld::PhysicsWorld(PhysicsSettings settings)
    : settings_(settings), spatialHash_(settings.spatialCellSize) {}

void PhysicsWorld::resetAccumulatedTime() { accumulator_ = 0.0f; }

void PhysicsWorld::step(World& world, float dt) {
  accumulator_ += dt;
  const float step = settings_.fixedTimestep;
  int safety = 0;

  while (accumulator_ >= step && safety < 8) {
    auto tStartSubstep = std::chrono::high_resolution_clock::now();

    // 1. Integration
    auto t0 = std::chrono::high_resolution_clock::now();
    integrate(world, step);
    auto t1 = std::chrono::high_resolution_clock::now();
    profileStats_.integrationUs =
        std::chrono::duration<float, std::micro>(t1 - t0).count();

    // 2. Velocity Iterations (Constraints + Contacts)
    auto tConstraintStart = std::chrono::high_resolution_clock::now();
    for (int iter = 0; iter < settings_.velocityIterations; ++iter) {
      constraintSolver_.solveVelocityConstraints(world, distanceJoints_, springJoints_, mouseSpring_, step);
      detectAndResolve(world, step);
    }
    auto tConstraintEnd = std::chrono::high_resolution_clock::now();
    profileStats_.constraintSolverUs =
        std::chrono::duration<float, std::micro>(tConstraintEnd - tConstraintStart).count();

    // 3. Position Iterations (Joints)
    for (int iter = 0; iter < settings_.positionIterations; ++iter) {
      constraintSolver_.solvePositionConstraints(world, distanceJoints_);
    }

    auto tEndSubstep = std::chrono::high_resolution_clock::now();
    profileStats_.totalSubstepUs =
        std::chrono::duration<float, std::micro>(tEndSubstep - tStartSubstep).count();

    simulationTime_ += step;
    accumulator_ -= step;
    ++safety;
  }
}

void PhysicsWorld::integrate(World& world, float dt) {
  for (auto& rec : world.entitiesMutable()) {
    if (!rec.active) continue;
    integrateRigidBody(rec.rigidBody, rec.transform, settings_.gravity, dt);
  }
}

void PhysicsWorld::detectAndResolve(World& world, float dt) {
  debugContacts_.clear();
  collisionCountLastFrame_ = 0;
  auto& records = world.entitiesMutable();

  std::vector<EntityId> ids;
  std::vector<glm::vec3> positions;
  std::vector<float> radii;
  ids.reserve(records.size());
  positions.reserve(records.size());
  radii.reserve(records.size());

  for (std::size_t i = 0; i < records.size(); ++i) {
    const auto& rec = records[i];
    if (!rec.active) continue;
    ids.push_back(static_cast<EntityId>(i));
    positions.push_back(rec.transform.position);
    float r = (rec.collider.shape == ColliderShape::Sphere)
                  ? rec.collider.radius
                  : glm::length(rec.collider.halfExtents);
    radii.push_back(r);
  }

  // Broadphase: Spatial Hash
  auto tBroadStart = std::chrono::high_resolution_clock::now();
  spatialHash_.clear();
  for (std::size_t i = 0; i < ids.size(); ++i) {
    spatialHash_.insert(static_cast<EntityId>(i), positions[i], radii[i]);
  }
  auto pairs = spatialHash_.queryPairs(ids, positions, radii);
  auto tBroadEnd = std::chrono::high_resolution_clock::now();
  profileStats_.broadphaseUs =
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
    resolveContact(recA.rigidBody, recA.transform, recB.rigidBody, recB.transform, *contact);

    // Position correction to prevent sinking/penetration
    const float slop = 0.003f;
    const float percent = 0.65f;
    float invMassSum = recA.rigidBody.inverseMass + recB.rigidBody.inverseMass;
    if (invMassSum > 0.0f) {
      glm::vec3 correction =
          contact->normal * (std::max(contact->penetration - slop, 0.0f) * percent / invMassSum);
      if (!recA.rigidBody.isStatic) recA.transform.position -= correction * recA.rigidBody.inverseMass;
      if (!recB.rigidBody.isStatic) recB.transform.position += correction * recB.rigidBody.inverseMass;
    }
  }

  auto tNarrowEnd = std::chrono::high_resolution_clock::now();
  profileStats_.narrowphaseUs =
      std::chrono::duration<float, std::micro>(tNarrowEnd - tNarrowStart).count();
}

}  // namespace simforge
