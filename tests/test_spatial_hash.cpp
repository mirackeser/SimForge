#include "physics/SpatialHash.hpp"

#include <gtest/gtest.h>

using namespace simforge;

TEST(SpatialHash, FindsClosePair) {
  SpatialHash hash(2.0f);
  std::vector<EntityId> ids = {0, 1};
  std::vector<glm::vec3> positions = {{0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}};
  std::vector<float> radii = {1.0f, 1.0f};

  hash.insert(0, positions[0], radii[0]);
  hash.insert(1, positions[1], radii[1]);

  auto pairs = hash.queryPairs(ids, positions, radii);
  ASSERT_EQ(pairs.size(), 1u);
  EXPECT_TRUE((pairs[0].a == 0 && pairs[0].b == 1) || (pairs[0].a == 1 && pairs[0].b == 0));
}
