#include "physics/SpatialHash.hpp"

#include <unordered_set>

namespace simforge {

SpatialHash::SpatialHash(float cellSize) : cellSize_(cellSize) {}

void SpatialHash::clear() { grid_.clear(); }

void SpatialHash::insert(EntityId id, const glm::vec3& position, float radius) {
  glm::ivec3 minCell = cellCoord(position - glm::vec3(radius));
  glm::ivec3 maxCell = cellCoord(position + glm::vec3(radius));
  for (int x = minCell.x; x <= maxCell.x; ++x) {
    for (int y = minCell.y; y <= maxCell.y; ++y) {
      for (int z = minCell.z; z <= maxCell.z; ++z) {
        grid_[CellKey{x, y, z}].push_back(id);
      }
    }
  }
}

std::vector<CollisionPair> SpatialHash::queryPairs(const std::vector<EntityId>& ids,
                                                   const std::vector<glm::vec3>& positions,
                                                   const std::vector<float>& radii) const {
  std::vector<CollisionPair> pairs;
  std::unordered_set<std::uint64_t> seen;

  auto pairKey = [](EntityId a, EntityId b) {
    if (a > b) std::swap(a, b);
    return (static_cast<std::uint64_t>(a) << 32) | b;
  };

  for (const auto& [cell, cellIds] : grid_) {
    for (std::size_t i = 0; i < cellIds.size(); ++i) {
      for (std::size_t j = i + 1; j < cellIds.size(); ++j) {
        EntityId a = cellIds[i];
        EntityId b = cellIds[j];
        auto key = pairKey(a, b);
        if (seen.count(key)) continue;
        seen.insert(key);

        float reach = radii[a] + radii[b];
        glm::vec3 delta = positions[b] - positions[a];
        if (glm::dot(delta, delta) <= reach * reach) {
          pairs.push_back(CollisionPair{a, b});
        }
      }
    }
  }

  (void)ids;
  return pairs;
}

glm::ivec3 SpatialHash::cellCoord(const glm::vec3& p) const {
  return glm::ivec3(static_cast<int>(std::floor(p.x / cellSize_)),
                    static_cast<int>(std::floor(p.y / cellSize_)),
                    static_cast<int>(std::floor(p.z / cellSize_)));
}

std::size_t SpatialHash::CellKeyHash::operator()(const CellKey& key) const {
  std::size_t h = static_cast<std::size_t>(key.x) * 73856093u;
  h ^= static_cast<std::size_t>(key.y) * 19349663u;
  h ^= static_cast<std::size_t>(key.z) * 83492791u;
  return h;
}

}  // namespace simforge
