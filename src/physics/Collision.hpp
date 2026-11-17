#pragma once

#include "core/Types.hpp"
#include "ecs/Components.hpp"

#include <glm/glm.hpp>
#include <optional>

namespace simforge {

struct ContactManifold {
  // Normal points from shape A toward shape B; resolution moves A against it.
  glm::vec3 normal{0.0f, 1.0f, 0.0f};
  float penetration{0.0f};
  glm::vec3 point{0.0f};
};

struct CollisionPair {
  EntityId a;
  EntityId b;
};

struct Ray {
  glm::vec3 origin{0.0f};
  glm::vec3 direction{0.0f, 0.0f, -1.0f};
};

struct RaycastHit {
  EntityId entityId{kInvalidEntity};
  glm::vec3 point{0.0f};
  glm::vec3 normal{0.0f};
  float distance{0.0f};
};

std::optional<ContactManifold> collideSphereSphere(const TransformComponent& a, const ColliderComponent& ca,
                                                    const TransformComponent& b, const ColliderComponent& cb);

std::optional<ContactManifold> collideSphereBox(const TransformComponent& sphere, const ColliderComponent& cs,
                                                  const TransformComponent& box, const ColliderComponent& cb);

std::optional<ContactManifold> collideBoxBox(const TransformComponent& a, const ColliderComponent& ca,
                                               const TransformComponent& b, const ColliderComponent& cb);

void resolveContact(RigidBodyComponent& bodyA, TransformComponent& transformA,
                    RigidBodyComponent& bodyB, TransformComponent& transformB,
                    const ContactManifold& contact, bool enableFriction = true, bool enableRotation = true);

void resolveContact(RigidBodyComponent& bodyA, RigidBodyComponent& bodyB, const ContactManifold& contact);

void integrateRigidBody(RigidBodyComponent& body, TransformComponent& transform, const glm::vec3& gravity,
                        float dt, bool enableRotation = true);

bool intersectRaySphere(const Ray& ray, const glm::vec3& center, float radius, float& tHit, glm::vec3& hitNormal);

bool intersectRayOBB(const Ray& ray, const glm::vec3& center, const glm::quat& rotation,
                     const glm::vec3& halfExtents, float& tHit, glm::vec3& hitNormal);

}  // namespace simforge
