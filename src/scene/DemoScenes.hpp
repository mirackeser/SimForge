#pragma once

#include "ecs/World.hpp"
#include "physics/PhysicsWorld.hpp"

namespace simforge {

enum class DemoScene {
  DominoStack,
  NewtonsCradle,
  JengaTower,
  RopeBridge,
  ParticleRain,
  MixedStress,
  MaterialsLab
};

void loadDemoScene(World& world, PhysicsWorld& physics, DemoScene scene);

const char* demoSceneName(DemoScene scene);

}  // namespace simforge
