#include "app/Application.hpp"

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <algorithm>
#include <fstream>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/random.hpp>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

namespace simforge {

Application::~Application() { shutdown(); }

bool Application::init() {
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return false;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

  window_ = glfwCreateWindow(1600, 900, "SimForge | Physics Studio", nullptr, nullptr);
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

  if (!initUi(true)) return false;

  loadDemoScene(world_, physics_, activeScene_);
  lastFrameTime_ = glfwGetTime();
  return true;
}

bool Application::initUi(bool nativeInput) {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();

  ImGuiStyle& style = ImGui::GetStyle();
  ImGui::GetIO().FontGlobalScale = 1.1f;
  style.WindowPadding = ImVec2(16, 14);
  style.ItemSpacing = ImVec2(8, 9);
  style.WindowRounding = 8.0f;
  style.FrameRounding = 4.0f;
  style.GrabRounding = 4.0f;
  style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.1f, 0.13f, 0.16f, 1.0f);
  style.Colors[ImGuiCol_CheckMark] = ImVec4(0.8f, 0.69f, 0.43f, 1.0f);
  style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.4f, 0.58f, 0.65f, 1.0f);
  style.Colors[ImGuiCol_WindowBg] = ImVec4(0.055f, 0.07f, 0.09f, 0.98f);
  style.Colors[ImGuiCol_Header] = ImVec4(0.13f, 0.18f, 0.23f, 1.0f);
  style.Colors[ImGuiCol_Button] = ImVec4(0.16f, 0.24f, 0.3f, 1.0f);
  style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.23f, 0.36f, 0.43f, 1.0f);

  if (nativeInput && !ImGui_ImplGlfw_InitForOpenGL(window_, true)) return false;
  if (!ImGui_ImplOpenGL3_Init("#version 330")) return false;

  return true;
}

bool Application::capturePreview(const std::string& path, void* (*loader)(const char*)) {
  constexpr int width = 1600, height = 900;
  if (!renderer_.init(nullptr, loader) || !initUi(false)) return false;
  ImGui::GetIO().IniFilename = nullptr;
  ImGui::GetIO().DisplaySize = ImVec2(width, height);
  ImGui::GetIO().DeltaTime = physics_.settings().fixedTimestep;
  loadDemoScene(world_, physics_, activeScene_);
  for (int frame = 0; frame < 45; ++frame) {
    physics_.stepFixed(world_);
    renderer_.onFramebufferResize(width, height);
    renderer_.beginFrame(camera_, static_cast<float>(width) / height);
    renderer_.drawWorld(world_);
    renderer_.drawJoints(world_, physics_.distanceJoints(), physics_.springJoints(),
                         physics_.mouseSpring());
    renderer_.drawTrails(world_);
    renderer_.flushLines();
    drawUi();
  }
  std::vector<unsigned char> pixels(width * height * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
  if (glGetError() != GL_NO_ERROR) return false;
  std::ofstream output(path, std::ios::binary);
  if (!output) return false;
  output << "P6\n" << width << ' ' << height << "\n255\n";
  for (int row = height - 1; row >= 0; --row) {
    output.write(reinterpret_cast<const char*>(pixels.data() + row * width * 3), width * 3);
  }
  output.flush();
  return static_cast<bool>(output);
}

void Application::shutdown() {
  if (ImGui::GetCurrentContext()) {
    if (ImGui::GetIO().BackendRendererUserData) ImGui_ImplOpenGL3_Shutdown();
    if (ImGui::GetIO().BackendPlatformUserData) ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
  }
  renderer_.shutdown();
  if (window_) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
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
  // Always release the spring, even when the pointer has entered an ImGui panel.
  if (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE) {
    isDraggingEntity_ = false;
    physics_.mouseSpring().active = false;
  }
  if (io.WantCaptureMouse) return;

  // GLFW cursor positions are in window coordinates, including on Retina displays.
  int fbW, fbH;
  glfwGetWindowSize(window_, &fbW, &fbH);
  if (fbW <= 0 || fbH <= 0) return;

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

  const bool spacePressed = glfwGetKey(window_, GLFW_KEY_SPACE) == GLFW_PRESS;
  if (spacePressed && !spaceWasPressed_ && !ImGui::GetIO().WantCaptureKeyboard) {
    paused_ = !paused_;
    physics_.resetAccumulatedTime();
  }
  spaceWasPressed_ = spacePressed;

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
  if (glfwGetKey(window_, GLFW_KEY_EQUAL) == GLFW_PRESS ||
      glfwGetKey(window_, GLFW_KEY_KP_ADD) == GLFW_PRESS) {
    camera_.distance = glm::max(3.0f, camera_.distance - dt * 10.0f);
  }
  if (glfwGetKey(window_, GLFW_KEY_MINUS) == GLFW_PRESS ||
      glfwGetKey(window_, GLFW_KEY_KP_SUBTRACT) == GLFW_PRESS) {
    camera_.distance = glm::min(70.0f, camera_.distance + dt * 10.0f);
  }
}

void Application::drawUi() {
  ImGui_ImplOpenGL3_NewFrame();
  if (window_) ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  const auto display = ImGui::GetIO().DisplaySize;
  ImGui::SetNextWindowPos(ImVec2(std::max(16.0f, display.x - 316.0f), 16), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(300, std::max(200.0f, display.y - 32.0f)), ImGuiCond_Always);

  ImGui::Begin("SIMFORGE", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
  ImGui::TextColored(ImVec4(0.8f, 0.69f, 0.43f, 1.0f), "PHYSICS STUDIO");
  ImGui::TextDisabled("Real-time rigid body laboratory");
  ImGui::Separator();

  int sceneIndex = static_cast<int>(activeScene_);
  const char* sceneList =
      "Domino Chain & Towers\0Newton's Cradle (Constraints)\0Jenga Stacking Tower\0Suspended Rope "
      "Bridge\0Particle Rain (Broadphase)\0Mixed Stress Sandbox\0Physics Studio\0";

  ImGui::TextDisabled("SCENE");
  ImGui::SetNextItemWidth(-1);
  if (ImGui::Combo("##scene", &sceneIndex, sceneList)) {
    activeScene_ = static_cast<DemoScene>(sceneIndex);
    loadDemoScene(world_, physics_, activeScene_);
    selectedEntity_ = kInvalidEntity;
    isDraggingEntity_ = false;
    renderer_.clearTrails();
    stepOnce_ = false;
  }

  if (ImGui::Button(paused_ ? "Resume (Space)" : "Pause (Space)", ImVec2(128, 0))) {
    paused_ = !paused_;
    physics_.resetAccumulatedTime();
  }
  ImGui::SameLine();
  if (ImGui::Button("Step Frame", ImVec2(128, 0))) {
    paused_ = true;
    stepOnce_ = true;
  }
  if (ImGui::Button("Reset Scene", ImVec2(-1, 0))) {
    loadDemoScene(world_, physics_, activeScene_);
    selectedEntity_ = kInvalidEntity;
    isDraggingEntity_ = false;
    renderer_.clearTrails();
    stepOnce_ = false;
  }

  ImGui::Spacing();
  if (ImGui::Button("+ Sphere", ImVec2(128, 0))) {
    spawnRandomSphere();
  }
  ImGui::SameLine();
  if (ImGui::Button("+ Box", ImVec2(128, 0))) {
    spawnRandomBox();
  }

  ImGui::Separator();

  auto& settings = physics_.settings();
  if (ImGui::CollapsingHeader("Physics & Solver Settings")) {
    ImGui::TextUnformatted("Gravity (m/s2)");
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderFloat3("##gravity", &settings.gravity.x, -25.0f, 5.0f);

    float fixedDtMs = settings.fixedTimestep * 1000.0f;
    ImGui::TextUnformatted("Fixed timestep");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::SliderFloat("##timestep", &fixedDtMs, 1000.0f / 240.0f, 1000.0f / 30.0f,
                           "%.2f ms")) {
      settings.fixedTimestep = fixedDtMs / 1000.0f;
    }

    ImGui::TextUnformatted("Velocity iterations");
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderInt("##velocity-iterations", &settings.velocityIterations, 1, 20);
    ImGui::TextUnformatted("Position iterations");
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderInt("##position-iterations", &settings.positionIterations, 1, 10);
    ImGui::Checkbox("Friction", &settings.enableFriction);
    ImGui::Checkbox("Rotational Dynamics", &settings.enableRotationalDynamics);
  }

  if (ImGui::CollapsingHeader("Debug Overlays")) {
    if (ImGui::Checkbox("Motion Trails", &showTrails_)) renderer_.clearTrails();
    ImGui::Checkbox("Reference Grid", &showGrid_);
    ImGui::Checkbox("Contact Points & Normals", &showContacts_);
    ImGui::Checkbox("Distance & Spring Joints", &showJoints_);
    ImGui::Checkbox("Spatial Hash Grid Cells", &showSpatialGrid_);
  }

  if (ImGui::CollapsingHeader("Performance Profiler", ImGuiTreeNodeFlags_DefaultOpen)) {
    const auto& prof = physics_.profileStats();
    ImGui::Text("Simulation: %.3f s", physics_.simulationTime());
    ImGui::Text("Active Entities: %zu", world_.entities().size());
    ImGui::Text("Contacts (Last Substep): %d", physics_.collisionCountLastFrame());
    ImGui::Text("Active Joints: %zu",
                physics_.distanceJoints().size() + physics_.springJoints().size());
    ImGui::Separator();
    ImGui::Text("Integration:      %6.1f us", prof.integrationUs);
    ImGui::Text("Broadphase:       %6.1f us", prof.broadphaseUs);
    ImGui::Text("Contacts & solve: %6.1f us", prof.narrowphaseUs);
    ImGui::Text("Joints:           %6.1f us", prof.constraintSolverUs);
    ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "Total substep:    %6.1f us",
                       prof.totalSubstepUs);
    ImGui::Separator();
    if (window_)
      ImGui::Text("FPS: %.1f (%.2f ms)", fpsSmoothed_, 1000.0f / std::max(fpsSmoothed_, 1.0f));
    else
      ImGui::TextDisabled("Offline capture");
  }

  ImGui::Separator();
  ImGui::TextDisabled("Controls:");
  ImGui::BulletText("LMB: Grab / throw");
  ImGui::BulletText("RMB: Orbit camera");
  ImGui::BulletText("+ / - : Zoom");
  ImGui::End();

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
      ImGui::Text("Linear Velocity: (%.2f, %.2f, %.2f)", rb->velocity.x, rb->velocity.y,
                  rb->velocity.z);
      ImGui::Text("Angular Velocity: (%.2f, %.2f, %.2f)", rb->angularVelocity.x,
                  rb->angularVelocity.y, rb->angularVelocity.z);

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
    shutdown();
    return 1;
  }

  while (!glfwWindowShouldClose(window_)) {
    double now = glfwGetTime();
    float dt = static_cast<float>(now - lastFrameTime_);
    lastFrameTime_ = now;
    fpsSmoothed_ = fpsSmoothed_ * 0.92f + (1.0f / glm::max(dt, 1e-4f)) * 0.08f;

    glfwPollEvents();
    processInput(dt);

    if (stepOnce_) {
      physics_.resetAccumulatedTime();
      physics_.stepFixed(world_);
      stepOnce_ = false;
    } else if (!paused_) {
      physics_.step(world_, dt);
    }

    int fbW, fbH;
    glfwGetFramebufferSize(window_, &fbW, &fbH);
    if (fbW <= 0 || fbH <= 0) {
      glfwWaitEventsTimeout(0.05);
      continue;
    }
    renderer_.onFramebufferResize(fbW, fbH);
    const float aspect = fbW / static_cast<float>(fbH);
    renderer_.beginFrame(camera_, aspect);
    if (showGrid_) renderer_.drawGrid(50.0f, 50);
    renderer_.drawWorld(world_, selectedEntity_);
    if (showTrails_) renderer_.drawTrails(world_);

    if (showContacts_) {
      renderer_.drawContacts(physics_.debugContacts());
    }
    if (showJoints_) {
      renderer_.drawJoints(world_, physics_.distanceJoints(), physics_.springJoints(),
                           physics_.mouseSpring());
    }
    if (showSpatialGrid_) {
      renderer_.drawSpatialHashCells(physics_.spatialHash(), physics_.spatialHash().cellSize());
    }

    renderer_.flushLines();
    drawUi();
    renderer_.endFrame();
  }

  shutdown();
  return 0;
}

}  // namespace simforge
