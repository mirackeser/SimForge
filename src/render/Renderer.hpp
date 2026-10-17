#pragma once

#include "ecs/World.hpp"
#include "physics/Collision.hpp"
#include "physics/Constraint.hpp"
#include "physics/SpatialHash.hpp"
#include "render/Shader.hpp"

#include <glm/glm.hpp>
#include <vector>

struct GLFWwindow;

namespace simforge {

struct OrbitCamera {
  glm::vec3 target{0.0f, 2.0f, 0.0f};
  float distance{18.0f};
  float yaw{0.6f};
  float pitch{0.35f};
  float fovDegrees{45.0f};

  glm::mat4 viewMatrix() const;
  glm::mat4 projectionMatrix(float aspect) const;
  glm::vec3 position() const;
};

class Renderer {
 public:
  bool init(GLFWwindow* window);
  void shutdown();

  void beginFrame(const OrbitCamera& camera, float aspect);
  void drawWorld(const World& world, EntityId selectedEntity = kInvalidEntity);
  void drawGrid(float size, int divisions);
  void endFrame();

  void onFramebufferResize(int width, int height);

  Ray screenPointToRay(double mouseX, double mouseY, int screenWidth, int screenHeight,
                       const OrbitCamera& camera) const;

  // Debug rendering
  void drawLine(const glm::vec3& start, const glm::vec3& end, const glm::vec3& color);
  void drawContacts(const std::vector<ContactManifold>& contacts);
  void drawJoints(const World& world, const std::vector<DistanceJoint>& distanceJoints,
                  const std::vector<SpringJoint>& springJoints, const MouseSpring& mouseSpring);
  void drawSpatialHashCells(const SpatialHash& spatialHash, float cellSize);
  void flushLines();

 private:
  void createUnitMeshes();

  GLFWwindow* window_{nullptr};
  Shader shader_;
  Shader gridShader_;
  Shader lineShader_;

  glm::mat4 lastView_{1.0f};
  glm::mat4 lastProjection_{1.0f};
  glm::vec3 lastCameraPos_{0.0f};

  unsigned int cubeVao_{0};
  unsigned int cubeVbo_{0};
  unsigned int cubeEbo_{0};
  unsigned int cubeIndexCount_{0};

  unsigned int sphereVao_{0};
  unsigned int sphereVbo_{0};
  unsigned int sphereEbo_{0};
  unsigned int sphereIndexCount_{0};

  unsigned int gridVao_{0};
  unsigned int gridVbo_{0};
  unsigned int gridVertexCount_{0};

  unsigned int lineVao_{0};
  unsigned int lineVbo_{0};
  std::vector<float> lineVertices_;

  int framebufferWidth_{1280};
  int framebufferHeight_{720};
};

}  // namespace simforge
