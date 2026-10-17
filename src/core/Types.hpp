#pragma once

#include <cstdint>
#include <limits>

namespace simforge {

using EntityId = std::uint32_t;
inline constexpr EntityId kInvalidEntity = std::numeric_limits<EntityId>::max();

}  // namespace simforge
