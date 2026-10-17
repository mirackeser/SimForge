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
