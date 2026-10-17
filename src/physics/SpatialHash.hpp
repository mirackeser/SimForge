#pragma once

#include "physics/Collision.hpp"

#include <glm/glm.hpp>
#include <unordered_map>
#include <vector>

namespace simforge {

class SpatialHash {
 public:
  explicit SpatialHash(float cellSize = 2.0f);

  void clear();
  void insert(EntityId id, const glm::vec3& position, float radius);

  std::vector<CollisionPair> queryPairs(const std::vector<EntityId>& ids,
                                        const std::vector<glm::vec3>& positions,
                                        const std::vector<float>& radii) const;

  struct CellKey {
    int x;
    int y;
    int z;

    bool operator==(const CellKey& other) const { return x == other.x && y == other.y && z == other.z; }
  };

  struct CellKeyHash {
    std::size_t operator()(const CellKey& key) const;
  };

  const std::unordered_map<CellKey, std::vector<EntityId>, CellKeyHash>& grid() const { return grid_; }
  float cellSize() const { return cellSize_; }

 private:
  glm::ivec3 cellCoord(const glm::vec3& p) const;

  float cellSize_;
  std::unordered_map<CellKey, std::vector<EntityId>, CellKeyHash> grid_;
};

}  // namespace simforge
