#pragma once

#include "ecs/Components.hpp"
#include "ecs/Entity.hpp"

#include <string>
#include <vector>

namespace simforge {

class World {
 public:
  Entity createEntity(const std::string& name = {});

  void destroyEntity(Entity entity);

  bool alive(Entity entity) const;

  TransformComponent* transform(Entity entity);
  RigidBodyComponent* rigidBody(Entity entity);
  ColliderComponent* collider(Entity entity);
  RenderComponent* render(Entity entity);
  TagComponent* tag(Entity entity);

  const TransformComponent* transform(Entity entity) const;
  const RigidBodyComponent* rigidBody(Entity entity) const;
  const ColliderComponent* collider(Entity entity) const;
  const RenderComponent* render(Entity entity) const;

  struct EntityRecord {
    Entity entity;
    TransformComponent transform;
    RigidBodyComponent rigidBody;
    ColliderComponent collider;
    RenderComponent render;
    TagComponent tag;
    bool active{true};
  };

  const std::vector<EntityRecord>& entities() const { return records_; }
  std::vector<EntityRecord>& entitiesMutable() { return records_; }

  void clear();

 private:
  EntityId nextId_{0};
  std::vector<EntityRecord> records_;
};

}  // namespace simforge
