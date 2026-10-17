#pragma once

#include "core/Types.hpp"

namespace simforge {

struct Entity {
  EntityId id{kInvalidEntity};

  bool valid() const { return id != kInvalidEntity; }
  explicit operator bool() const { return valid(); }
};

inline bool operator==(Entity a, Entity b) { return a.id == b.id; }
inline bool operator!=(Entity a, Entity b) { return !(a == b); }

}  // namespace simforge
