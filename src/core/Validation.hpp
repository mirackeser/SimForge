#pragma once

#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace simforge {

inline bool finite(const glm::vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

inline bool finite(const glm::quat& value) {
  return std::isfinite(value.w) && std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

}  // namespace simforge
