#include "physics/Collision.hpp"

#include <gtest/gtest.h>

using namespace simforge;

TEST(Collision, SphereSphereOverlap) {
  TransformComponent a;
  a.position = {0.0f, 0.0f, 0.0f};
  ColliderComponent ca;
  ca.shape = ColliderShape::Sphere;
  ca.radius = 1.0f;

  TransformComponent b;
  b.position = {1.5f, 0.0f, 0.0f};
  ColliderComponent cb = ca;

  auto contact = collideSphereSphere(a, ca, b, cb);
  ASSERT_TRUE(contact.has_value());
  EXPECT_GT(contact->penetration, 0.0f);
}

TEST(Collision, SphereBoxOverlap) {
  TransformComponent sphere;
  sphere.position = {0.0f, 0.0f, 0.0f};
  ColliderComponent cs;
  cs.shape = ColliderShape::Sphere;
  cs.radius = 1.0f;

  TransformComponent box;
  box.position = {1.2f, 0.0f, 0.0f};
  ColliderComponent cb;
  cb.shape = ColliderShape::Box;
  cb.halfExtents = {1.0f, 1.0f, 1.0f};

  auto contact = collideSphereBox(sphere, cs, box, cb);
  ASSERT_TRUE(contact.has_value());
  EXPECT_GT(contact->penetration, 0.0f);
}

TEST(Collision, CoincidentSphereCentersHaveUnitNormal) {
  TransformComponent transform;
  ColliderComponent collider;
  const auto contact = collideSphereSphere(transform, collider, transform, collider);
  ASSERT_TRUE(contact);
  EXPECT_NEAR(glm::length(contact->normal), 1.0f, 1e-6f);
  EXPECT_NEAR(contact->penetration, 1.0f, 1e-6f);
}

TEST(Collision, SphereInsideRotatedBoxEscapesThroughNearestFace) {
  TransformComponent box, sphere;
  ColliderComponent cb, cs;
  cb.shape = ColliderShape::Box;
  cb.halfExtents = glm::vec3(1.0f);
  box.rotation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0.0f, 0.0f, 1.0f));
  const glm::vec3 outward = box.rotation * glm::vec3(1.0f, 0.0f, 0.0f);
  sphere.position = outward * 0.8f;
  const auto contact = collideSphereBox(sphere, cs, box, cb);
  ASSERT_TRUE(contact);
  EXPECT_NEAR(glm::dot(contact->normal, outward), -1.0f, 1e-5f);
  EXPECT_NEAR(contact->penetration, 0.7f, 1e-5f);
  // Resolution moves A opposite the A -> B contact normal.
  EXPECT_GT(glm::dot(sphere.position - contact->normal * contact->penetration, outward), 1.0f);
}
