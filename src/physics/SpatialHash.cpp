#include "physics/SpatialHash.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

#include "core/Validation.hpp"

namespace simforge {

SpatialHash::SpatialHash(float cellSize) : cellSize_(cellSize) {
  if (!std::isfinite(cellSize) || cellSize <= 0.0f) {
    throw std::invalid_argument("Spatial cell size must be finite and positive");
  }
}

void SpatialHash::clear() {
  grid_.clear();
  entryCount_ = 0;
}

void SpatialHash::insert(EntityId id, const glm::vec3& position, float radius) {
  if (!std::isfinite(radius) || radius < 0.0f)
    throw std::invalid_argument("Invalid broadphase radius");
  insertAabb(id, position - glm::vec3(radius), position + glm::vec3(radius));
}

void SpatialHash::insertAabb(EntityId id, const glm::vec3& minimum, const glm::vec3& maximum) {
  if (!finite(minimum) || !finite(maximum) || glm::any(glm::greaterThan(minimum, maximum))) {
    throw std::invalid_argument("Invalid broadphase bounds");
  }
  glm::ivec3 minCell = cellCoord(minimum);
  glm::ivec3 maxCell = cellCoord(maximum);
  const double cells = (static_cast<double>(maxCell.x) - minCell.x + 1.0) *
                       (static_cast<double>(maxCell.y) - minCell.y + 1.0) *
                       (static_cast<double>(maxCell.z) - minCell.z + 1.0);
  if (cells > 262144.0 || entryCount_ + cells > 2000000.0) {
    throw std::length_error("Broadphase cell budget exceeded");
  }
  entryCount_ += static_cast<std::size_t>(cells);
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
  if (ids.size() != positions.size() || ids.size() != radii.size()) {
    throw std::invalid_argument("Broadphase arrays differ in size");
  }
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (!finite(positions[i]) || !std::isfinite(radii[i]) || radii[i] < 0.0f) {
      throw std::invalid_argument("Invalid broadphase query values");
    }
  }
  std::vector<CollisionPair> pairs;
  std::unordered_set<std::uint64_t> seen;
  std::size_t visits = 0;

  auto pairKey = [](EntityId a, EntityId b) {
    if (a > b) std::swap(a, b);
    return (static_cast<std::uint64_t>(a) << 32) | b;
  };

  for (const auto& [cell, cellIds] : grid_) {
    for (const auto id : cellIds) {
      if (id >= positions.size()) throw std::out_of_range("Broadphase index exceeds input size");
    }
    for (std::size_t i = 0; i < cellIds.size(); ++i) {
      for (std::size_t j = i + 1; j < cellIds.size(); ++j) {
        if (++visits > 2000000) throw std::length_error("Broadphase pair budget exceeded");
        EntityId a = cellIds[i];
        EntityId b = cellIds[j];
        if (a > b) std::swap(a, b);
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

  std::sort(pairs.begin(), pairs.end(), [](const CollisionPair& a, const CollisionPair& b) {
    return a.a < b.a || (a.a == b.a && a.b < b.b);
  });
  return pairs;
}

glm::ivec3 SpatialHash::cellCoord(const glm::vec3& p) const {
  glm::ivec3 result;
  for (int i = 0; i < 3; ++i) {
    const double value = std::floor(static_cast<double>(p[i]) / cellSize_);
    if (!std::isfinite(value) || value <= std::numeric_limits<int>::min() ||
        value >= std::numeric_limits<int>::max()) {
      throw std::out_of_range("Broadphase coordinate is outside the supported range");
    }
    result[i] = static_cast<int>(value);
  }
  return result;
}

std::size_t SpatialHash::CellKeyHash::operator()(const CellKey& key) const {
  std::size_t h = static_cast<std::size_t>(key.x) * 73856093u;
  h ^= static_cast<std::size_t>(key.y) * 19349663u;
  h ^= static_cast<std::size_t>(key.z) * 83492791u;
  return h;
}

}  // namespace simforge
