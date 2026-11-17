#include "ecs/World.hpp"

#include <algorithm>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <stdexcept>

#include "core/Validation.hpp"

namespace simforge {

glm::mat4 TransformComponent::matrix() const {
  glm::mat4 m(1.0f);
  m = glm::translate(m, position);
  m *= glm::mat4_cast(rotation);
  m = glm::scale(m, scale);
  return m;
}

void RigidBodyComponent::computeInertia(ColliderShape shape, float radius,
                                        const glm::vec3& halfExtents) {
  if (!std::isfinite(inverseMass) || inverseMass < 0.0f)
    throw std::invalid_argument("Invalid inverse mass");
  if (shape == ColliderShape::Sphere && (!std::isfinite(radius) || radius <= 0.0f)) {
    throw std::invalid_argument("Sphere radius must be positive and finite");
  }
  if (shape == ColliderShape::Box &&
      (!finite(halfExtents) || glm::any(glm::lessThanEqual(halfExtents, glm::vec3(0.0f))))) {
    throw std::invalid_argument("Box extents must be positive and finite");
  }
  if (isStatic || inverseMass <= 0.0f) {
    inverseInertiaTensorBody = glm::mat3(0.0f);
    inverseInertiaTensorWorld = glm::mat3(0.0f);
    return;
  }
  float mass = 1.0f / inverseMass;
  glm::mat3 inertia(0.0f);
  if (shape == ColliderShape::Sphere) {
    float i = 0.4f * mass * radius * radius;
    inertia[0][0] = i;
    inertia[1][1] = i;
    inertia[2][2] = i;
  } else {
    float x2 = halfExtents.x * halfExtents.x;
    float y2 = halfExtents.y * halfExtents.y;
    float z2 = halfExtents.z * halfExtents.z;
    // Standard solid cuboid formula: I_xx = (1/12)*m*(h^2 + d^2) for total lengths
    // With half extents: h_tot = 2*y, d_tot = 2*z => h_tot^2 + d_tot^2 = 4*(y^2 + z^2)
    // (4/12)*m = (1/3)*m
    inertia[0][0] = (1.0f / 3.0f) * mass * (y2 + z2);
    inertia[1][1] = (1.0f / 3.0f) * mass * (x2 + z2);
    inertia[2][2] = (1.0f / 3.0f) * mass * (x2 + y2);
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (!std::isfinite(inertia[axis][axis]) || inertia[axis][axis] < 1e-12f) {
      throw std::invalid_argument("Inertia is outside the supported numeric range");
    }
  }
  inverseInertiaTensorBody = glm::inverse(inertia);
  inverseInertiaTensorWorld = inverseInertiaTensorBody;
}

void RigidBodyComponent::updateInertiaWorld(const glm::quat& orientation) {
  if (isStatic || inverseMass <= 0.0f) {
    inverseInertiaTensorWorld = glm::mat3(0.0f);
    return;
  }
  glm::mat3 R = glm::mat3_cast(orientation);
  inverseInertiaTensorWorld = R * inverseInertiaTensorBody * glm::transpose(R);
}

Entity World::createEntity(const std::string& name) {
  if (records_.size() >= 10000 || nextId_ == kInvalidEntity) {
    throw std::length_error("Entity capacity exhausted");
  }
  EntityRecord rec;
  rec.entity = Entity{nextId_++};
  rec.tag.name = name;
  rec.render.label = name;
  records_.push_back(rec);
  return rec.entity;
}

void World::destroyEntity(Entity entity) {
  auto it = std::find_if(records_.begin(), records_.end(),
                         [&](const EntityRecord& r) { return r.entity.id == entity.id; });
  if (it != records_.end()) {
    it->active = false;
    *it = records_.back();
    records_.pop_back();
  }
}

bool World::alive(Entity entity) const {
  return std::any_of(records_.begin(), records_.end(),
                     [&](const EntityRecord& r) { return r.active && r.entity.id == entity.id; });
}

TransformComponent* World::transform(Entity entity) {
  for (auto& r : records_) {
    if (r.active && r.entity.id == entity.id) return &r.transform;
  }
  return nullptr;
}

RigidBodyComponent* World::rigidBody(Entity entity) {
  for (auto& r : records_) {
    if (r.active && r.entity.id == entity.id) return &r.rigidBody;
  }
  return nullptr;
}

ColliderComponent* World::collider(Entity entity) {
  for (auto& r : records_) {
    if (r.active && r.entity.id == entity.id) return &r.collider;
  }
  return nullptr;
}

RenderComponent* World::render(Entity entity) {
  for (auto& r : records_) {
    if (r.active && r.entity.id == entity.id) return &r.render;
  }
  return nullptr;
}

TagComponent* World::tag(Entity entity) {
  for (auto& r : records_) {
    if (r.active && r.entity.id == entity.id) return &r.tag;
  }
  return nullptr;
}

const TransformComponent* World::transform(Entity entity) const {
  return const_cast<World*>(this)->transform(entity);
}

const RigidBodyComponent* World::rigidBody(Entity entity) const {
  return const_cast<World*>(this)->rigidBody(entity);
}

const ColliderComponent* World::collider(Entity entity) const {
  return const_cast<World*>(this)->collider(entity);
}

const RenderComponent* World::render(Entity entity) const {
  return const_cast<World*>(this)->render(entity);
}

void World::clear() { records_.clear(); }

}  // namespace simforge
