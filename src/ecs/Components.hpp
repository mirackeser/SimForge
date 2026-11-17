#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <string>

namespace simforge {

enum class ColliderShape { Sphere, Box };

struct TransformComponent {
  glm::vec3 position{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 scale{1.0f};

  glm::mat4 matrix() const;
};

struct RigidBodyComponent {
  glm::vec3 velocity{0.0f};
  glm::vec3 angularVelocity{0.0f};
  float inverseMass{1.0f};
  glm::mat3 inverseInertiaTensorBody{1.0f};
  glm::mat3 inverseInertiaTensorWorld{1.0f};
  float restitution{0.35f};
  float staticFriction{0.5f};
  float dynamicFriction{0.35f};
  float linearDamping{0.015f};
  float angularDamping{0.035f};
  bool isStatic{false};
  bool useGravity{true};

  void computeInertia(ColliderShape shape, float radius, const glm::vec3& halfExtents);
  void updateInertiaWorld(const glm::quat& orientation);
};

struct ColliderComponent {
  ColliderShape shape{ColliderShape::Sphere};
  glm::vec3 halfExtents{0.5f};  // box half-size
  float radius{0.5f};           // sphere
};

struct RenderComponent {
  glm::vec3 color{0.85f, 0.55f, 0.25f};
  float metallic{0.0f};
  float roughness{0.5f};
  int surface{0};  // 0: uniform, 1: stone, 2: wood
  std::string label;
};

struct TagComponent {
  std::string name;
};

}  // namespace simforge
