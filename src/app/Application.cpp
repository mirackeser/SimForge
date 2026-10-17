#include "app/Application.hpp"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/random.hpp>
#include <algorithm>
#include <iostream>
#include <random>

namespace simforge {

bool Application::init() {
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return false;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window_ = glfwCreateWindow(1360, 780, "SimForge — Advanced Real-Time Physics Sandbox", nullptr, nullptr);
  if (!window_) {
    std::cerr << "Failed to create window\n";
    glfwTerminate();
    return false;
  }

  glfwSetWindowUserPointer(window_, this);
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  if (!renderer_.init(window_)) {
    std::cerr << "Failed to initialize renderer\n";
    return false;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();

  // Custom polished ImGui styling
  ImGuiStyle& style = ImGui::GetStyle();
  style.WindowRounding = 6.0f;
  style.FrameRounding = 4.0f;
  style.GrabRounding = 4.0f;
  style.Colors[ImGuiCol_WindowBg] = ImVec4(0.10f, 0.12f, 0.16f, 0.92f);
  style.Colors[ImGuiCol_Header] = ImVec4(0.20f, 0.35f, 0.55f, 0.85f);
  style.Colors[ImGuiCol_Button] = ImVec4(0.22f, 0.40f, 0.65f, 0.85f);
  style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.28f, 0.48f, 0.78f, 1.0f);

  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  loadDemoScene(world_, physics_, activeScene_);
  lastFrameTime_ = glfwGetTime();
  return true;
}

void Application::shutdown() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  renderer_.shutdown();
  if (window_) {
    glfwDestroyWindow(window_);
  }
  glfwTerminate();
}

void Application::spawnRandomSphere() {
  static std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<float> dist(-4.0f, 4.0f);
  std::uniform_real_distribution<float> rad(0.25f, 0.55f);
  float r = rad(rng);

  Entity e = world_.createEntity("spawned_sphere");
  auto* t = world_.transform(e);
  auto* rb = world_.rigidBody(e);
  auto* c = world_.collider(e);
  auto* rend = world_.render(e);

  t->position = glm::vec3(dist(rng), 9.0f, dist(rng));
  c->shape = ColliderShape::Sphere;
  c->radius = r;

  float mass = r * r * r * 8.0f;
  rb->inverseMass = 1.0f / mass;
  rb->restitution = 0.5f;
  rb->staticFriction = 0.5f;
  rb->dynamicFriction = 0.35f;
  rb->computeInertia(ColliderShape::Sphere, r, glm::vec3(0.0f));

  rend->color = glm::vec3(0.95f, 0.45f, 0.22f);
}

void Application::spawnRandomBox() {
  static std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<float> dist(-4.0f, 4.0f);
  std::uniform_real_distribution<float> extent(0.3f, 0.6f);

  glm::vec3 h(extent(rng), extent(rng), extent(rng));
  Entity e = world_.createEntity("spawned_box");
  auto* t = world_.transform(e);
  auto* rb = world_.rigidBody(e);
  auto* c = world_.collider(e);
  auto* rend = world_.render(e);

  t->position = glm::vec3(dist(rng), 9.0f, dist(rng));
  c->shape = ColliderShape::Box;
  c->halfExtents = h;

  float mass = h.x * h.y * h.z * 16.0f;
  rb->inverseMass = 1.0f / mass;
  rb->restitution = 0.25f;
  rb->staticFriction = 0.6f;
  rb->dynamicFriction = 0.4f;
  rb->computeInertia(ColliderShape::Box, 0.0f, h);

  rend->color = glm::vec3(0.35f, 0.75f, 0.95f);
}

void Application::handleMousePicking() {
  ImGuiIO& io = ImGui::GetIO();
  if (io.WantCaptureMouse) return;

  int fbW, fbH;
  glfwGetFramebufferSize(window_, &fbW, &fbH);

  double mouseX, mouseY;
  glfwGetCursorPos(window_, &mouseX, &mouseY);

  Ray ray = renderer_.screenPointToRay(mouseX, mouseY, fbW, fbH, camera_);

  // Left mouse button pressed: raycast pick
  if (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
    if (!isDraggingEntity_) {
      float closestT = std::numeric_limits<float>::max();
      EntityId hitEntity = kInvalidEntity;
      glm::vec3 hitPoint(0.0f);

      for (const auto& rec : world_.entities()) {
        if (!rec.active || rec.rigidBody.isStatic) continue;

        float tHit = 0.0f;
        glm::vec3 norm(0.0f);
        bool hit = false;

        if (rec.collider.shape == ColliderShape::Sphere) {
          hit = intersectRaySphere(ray, rec.transform.position, rec.collider.radius, tHit, norm);
        } else {
          hit = intersectRayOBB(ray, rec.transform.position, rec.transform.rotation,
                                rec.collider.halfExtents, tHit, norm);
        }

        if (hit && tHit < closestT && tHit > 0.1f) {
          closestT = tHit;
          hitEntity = rec.entity.id;
          hitPoint = ray.origin + ray.direction * tHit;
        }
      }

      if (hitEntity != kInvalidEntity) {
        selectedEntity_ = hitEntity;
        isDraggingEntity_ = true;
        dragDistance_ = closestT;

        auto* t = world_.transform(Entity{hitEntity});
        if (t) {
          MouseSpring ms;
          ms.entity = hitEntity;
          ms.localAnchor = glm::inverse(t->rotation) * (hitPoint - t->position);
          ms.worldTarget = hitPoint;
          ms.stiffness = 300.0f;
          ms.damping = 18.0f;
          ms.active = true;
          physics_.setMouseSpring(ms);
        }
      } else {
        selectedEntity_ = kInvalidEntity;
      }
    } else {
      // While dragging: update mouse target in world coordinates
      glm::vec3 target = ray.origin + ray.direction * dragDistance_;
      physics_.mouseSpring().worldTarget = target;
    }
  } else {
    // Mouse released
    if (isDraggingEntity_) {
      isDraggingEntity_ = false;
      physics_.mouseSpring().active = false;
    }
  }
}

void Application::processInput(float dt) {
  if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
    glfwSetWindowShouldClose(window_, true);
  }

  handleMousePicking();

  // Camera Orbit with Right Mouse Button
  if (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
    double x, y;
    glfwGetCursorPos(window_, &x, &y);
    if (!mouseOrbiting_) {
      mouseOrbiting_ = true;
      lastMouseX_ = x;
      lastMouseY_ = y;
    } else {
      camera_.yaw += static_cast<float>(x - lastMouseX_) * 0.005f;
      camera_.pitch += static_cast<float>(y - lastMouseY_) * 0.005f;
      camera_.pitch = glm::clamp(camera_.pitch, -1.25f, 1.25f);
      lastMouseX_ = x;
      lastMouseY_ = y;
    }
  } else {
    mouseOrbiting_ = false;
  }

  // Camera Zoom (+ / - or keypad)
  if (glfwGetKey(window_, GLFW_KEY_EQUAL) == GLFW_PRESS || glfwGetKey(window_, GLFW_KEY_KP_ADD) == GLFW_PRESS) {
    camera_.distance = glm::max(3.0f, camera_.distance - dt * 10.0f);
  }
  if (glfwGetKey(window_, GLFW_KEY_MINUS) == GLFW_PRESS || glfwGetKey(window_, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS) {
    camera_.distance = glm::min(70.0f, camera_.distance + dt * 10.0f);
  }
}

void Application::drawUi() {
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // 1. Main Control Panel
  ImGui::SetNextWindowPos(ImVec2(16, 16), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowSize(ImVec2(380, 560), ImGuiCond_FirstUseEver);

  ImGui::Begin("SimForge Control Panel");
  ImGui::TextColored(ImVec4(0.3f, 0.85f, 1.0f, 1.0f), "Advanced Rigid-Body Simulation Engine");
  ImGui::TextDisabled("Fixed Timestep • Spatial Hash • OBB SAT • Joints");
  ImGui::Separator();

  // Demo Scene Selector
  int sceneIndex = static_cast<int>(activeScene_);
  const char* sceneList =
      "Domino Chain & Towers\0Newton's Cradle (Constraints)\0Jenga Stacking Tower\0Suspended Rope Bridge\0Particle Rain (Broadphase)\0Mixed Stress Sandbox\0";

  if (ImGui::Combo("Scene", &sceneIndex, sceneList)) {
    activeScene_ = static_cast<DemoScene>(sceneIndex);
    loadDemoScene(world_, physics_, activeScene_);
    selectedEntity_ = kInvalidEntity;
    physics_.resetAccumulatedTime();
  }

  // Simulation State Controls
  if (ImGui::Button(paused_ ? "  Resume (Space)  " : "  Pause (Space)  ")) {
    paused_ = !paused_;
  }
  ImGui::SameLine();
  if (ImGui::Button("Step Frame")) {
    stepOnce_ = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Reset Scene")) {
    loadDemoScene(world_, physics_, activeScene_);
    selectedEntity_ = kInvalidEntity;
  }

  ImGui::Spacing();
  if (ImGui::Button("+ Spawn Sphere")) {
    spawnRandomSphere();
  }
  ImGui::SameLine();
  if (ImGui::Button("+ Spawn Box")) {
    spawnRandomBox();
  }

  ImGui::Separator();

  // Physics Parameters
  auto& settings = physics_.settings();
  if (ImGui::CollapsingHeader("Physics & Solver Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::SliderFloat3("Gravity", &settings.gravity.x, -25.0f, 5.0f);

    float fixedDtMs = settings.fixedTimestep * 1000.0f;
    if (ImGui::SliderFloat("Fixed dt (ms)", &fixedDtMs, 1000.0f / 240.0f, 1000.0f / 30.0f, "%.2f ms")) {
      settings.fixedTimestep = fixedDtMs / 1000.0f;
    }

    ImGui::SliderInt("Velocity Iterations", &settings.velocityIterations, 1, 20);
    ImGui::SliderInt("Position Iterations", &settings.positionIterations, 1, 10);
  }

  // Visual Overlays
  if (ImGui::CollapsingHeader("Debug Overlays", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Checkbox("Contact Points & Normals", &showContacts_);
    ImGui::Checkbox("Distance & Spring Joints", &showJoints_);
    ImGui::Checkbox("Spatial Hash Grid Cells", &showSpatialGrid_);
  }

  // Profiler Panel
  if (ImGui::CollapsingHeader("Performance Profiler", ImGuiTreeNodeFlags_DefaultOpen)) {
    const auto& prof = physics_.profileStats();
    ImGui::Text("Active Entities: %zu", world_.entities().size());
    ImGui::Text("Contacts (Last Substep): %d", physics_.collisionCountLastFrame());
    ImGui::Text("Active Joints: %zu", physics_.distanceJoints().size() + physics_.springJoints().size());
    ImGui::Separator();
    ImGui::Text("Integration:         %6.1f µs", prof.integrationUs);
    ImGui::Text("Broadphase Hash:     %6.1f µs", prof.broadphaseUs);
    ImGui::Text("Narrowphase & Solve: %6.1f µs", prof.narrowphaseUs);
    ImGui::Text("Constraint Solver:   %6.1f µs", prof.constraintSolverUs);
    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Total Substep:       %6.1f µs", prof.totalSubstepUs);
    ImGui::Separator();
    ImGui::Text("FPS: %.1f (%.2f ms)", fpsSmoothed_, 1000.0f / std::max(fpsSmoothed_, 1.0f));
  }

  ImGui::Separator();
  ImGui::TextDisabled("Controls:");
  ImGui::BulletText("LMB Drag: Grab and throw 3D objects (Mouse Spring)");
  ImGui::BulletText("RMB Drag: Orbit Camera");
  ImGui::BulletText("+ / - : Zoom In / Out");
  ImGui::End();

  // 2. Selected Entity Inspector
  if (selectedEntity_ != kInvalidEntity) {
    ImGui::SetNextWindowPos(ImVec2(16, 590), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(380, 160), ImGuiCond_FirstUseEver);

    ImGui::Begin("Selected Entity Inspector");
    auto* t = world_.transform(Entity{selectedEntity_});
    auto* rb = world_.rigidBody(Entity{selectedEntity_});
    auto* c = world_.collider(Entity{selectedEntity_});

    if (t && rb && c) {
      ImGui::Text("Entity ID: %u", selectedEntity_);
      ImGui::Text("Shape: %s", c->shape == ColliderShape::Sphere ? "Sphere" : "Oriented Box (OBB)");
      ImGui::Text("Position: (%.2f, %.2f, %.2f)", t->position.x, t->position.y, t->position.z);
      ImGui::Text("Linear Velocity: (%.2f, %.2f, %.2f)", rb->velocity.x, rb->velocity.y, rb->velocity.z);
      ImGui::Text("Angular Velocity: (%.2f, %.2f, %.2f)", rb->angularVelocity.x, rb->angularVelocity.y,
                  rb->angularVelocity.z);

      if (ImGui::Button("Upward Impulse")) {
        rb->velocity += glm::vec3(0.0f, 6.0f, 0.0f);
        rb->angularVelocity += glm::vec3(2.0f, 4.0f, 1.0f);
      }
      ImGui::SameLine();
      if (ImGui::Button("Deselect")) {
        selectedEntity_ = kInvalidEntity;
      }
    }
    ImGui::End();
  }

  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

int Application::run() {
  if (!init()) {
    return 1;
  }

  while (!glfwWindowShouldClose(window_)) {
    double now = glfwGetTime();
    float dt = static_cast<float>(now - lastFrameTime_);
    lastFrameTime_ = now;
    fpsSmoothed_ = fpsSmoothed_ * 0.92f + (1.0f / glm::max(dt, 1e-4f)) * 0.08f;

    glfwPollEvents();
    processInput(dt);

    if (!paused_ || stepOnce_) {
      physics_.step(world_, dt);
      stepOnce_ = false;
    }

    int fbW, fbH;
    glfwGetFramebufferSize(window_, &fbW, &fbH);
    renderer_.onFramebufferResize(fbW, fbH);
    float aspect = fbW / static_cast<float>(glm::max(fbH, 1));

    renderer_.beginFrame(camera_, aspect);
    renderer_.drawGrid(50.0f, 50);
    renderer_.drawWorld(world_, selectedEntity_);

    if (showContacts_) {
      renderer_.drawContacts(physics_.debugContacts());
    }
    if (showJoints_) {
      renderer_.drawJoints(world_, physics_.distanceJoints(), physics_.springJoints(), physics_.mouseSpring());
    }
    if (showSpatialGrid_) {
      renderer_.drawSpatialHashCells(physics_.spatialHash(), physics_.spatialHash().cellSize());
    }

    renderer_.endFrame();

    drawUi();
  }

  shutdown();
  return 0;
}

}  // namespace simforge
