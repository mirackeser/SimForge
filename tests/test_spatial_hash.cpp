#include <gtest/gtest.h>

#include "physics/SpatialHash.hpp"

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

TEST(SpatialHash, PairOrderIsCanonicalAndIndependentOfInsertionOrder) {
  SpatialHash forward(1.0f), reverse(1.0f);
  std::vector<EntityId> ids = {0, 1, 2};
  std::vector<glm::vec3> positions(3, glm::vec3(0.0f));
  std::vector<float> radii(3, 1.0f);
  for (EntityId i : ids) forward.insert(i, positions[i], radii[i]);
  for (auto it = ids.rbegin(); it != ids.rend(); ++it)
    reverse.insert(*it, positions[*it], radii[*it]);
  const auto a = forward.queryPairs(ids, positions, radii);
  const auto b = reverse.queryPairs(ids, positions, radii);
  ASSERT_EQ(a.size(), 3u);
  ASSERT_EQ(b.size(), a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_LT(a[i].a, a[i].b);
    EXPECT_EQ(a[i].a, b[i].a);
    EXPECT_EQ(a[i].b, b[i].b);
  }
}
