#pragma once

#include "ecs/World.hpp"
#include "physics/Collision.hpp"
#include "physics/PhysicsWorld.hpp"
#include "render/Renderer.hpp"
#include "scene/DemoScenes.hpp"

struct GLFWwindow;

namespace simforge {

class Application {
 public:
  ~Application();
  int run();
  bool capturePreview(const std::string& path, void* (*loader)(const char*));

 private:
  bool init();
  bool initUi(bool nativeInput);
  void shutdown();
  void processInput(float dt);
  void drawUi();
  void spawnRandomSphere();
  void spawnRandomBox();
  void handleMousePicking();

  GLFWwindow* window_{nullptr};
  World world_;
  PhysicsWorld physics_;
  Renderer renderer_;
  OrbitCamera camera_;

  DemoScene activeScene_{DemoScene::MaterialsLab};
  bool paused_{false};
  bool spaceWasPressed_{false};
  bool stepOnce_{false};
  double lastFrameTime_{0.0};
  float fpsSmoothed_{0.0f};

  // Camera Orbit
  bool mouseOrbiting_{false};
  double lastMouseX_{0.0};
  double lastMouseY_{0.0};

  // Interactive Mouse Picking / Spring Dragging
  EntityId selectedEntity_{kInvalidEntity};
  bool isDraggingEntity_{false};
  float dragDistance_{10.0f};

  // Debug Overlays
  bool showContacts_{false};
  bool showTrails_{true};
  bool showGrid_{false};
  bool showJoints_{true};
  bool showSpatialGrid_{false};
  bool showProfiler_{true};
  bool showEntityInspector_{true};
};

}  // namespace simforge
