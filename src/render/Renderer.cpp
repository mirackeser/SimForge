#include "render/Renderer.hpp"

#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include <algorithm>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

#include "render/Shaders.hpp"

namespace simforge {

namespace {

const char* kGridVertex = R"(#version 330 core
layout (location = 0) in vec3 aPos;
uniform mat4 uView;
uniform mat4 uProjection;
void main() {
  gl_Position = uProjection * uView * vec4(aPos, 1.0);
}
)";

const char* kGridFragment = R"(#version 330 core
out vec4 FragColor;
void main() {
  FragColor = vec4(0.22, 0.26, 0.32, 1.0);
}
)";

const char* kLineVertex = R"(#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

uniform mat4 uView;
uniform mat4 uProjection;

out vec3 vColor;

void main() {
  vColor = aColor;
  gl_Position = uProjection * uView * vec4(aPos, 1.0);
}
)";

const char* kLineFragment = R"(#version 330 core
in vec3 vColor;
out vec4 FragColor;

void main() {
  FragColor = vec4(vColor, 1.0);
}
)";

void uploadMesh(unsigned int& vao, unsigned int& vbo, unsigned int& ebo,
                const std::vector<float>& vertices, const std::vector<unsigned int>& indices,
                unsigned int& indexCount) {
  indexCount = static_cast<unsigned int>(indices.size());
  glGenVertexArrays(1, &vao);
  glGenBuffers(1, &vbo);
  glGenBuffers(1, &ebo);
  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  auto outward = indices;
  for (std::size_t i = 0; i + 2 < outward.size(); i += 3) {
    auto position = [&](unsigned int index) {
      return glm::vec3(vertices[index * 6], vertices[index * 6 + 1], vertices[index * 6 + 2]);
    };
    const auto a = outward[i], b = outward[i + 1], c = outward[i + 2];
    const glm::vec3 normal(vertices[a * 6 + 3], vertices[a * 6 + 4], vertices[a * 6 + 5]);
    if (glm::dot(glm::cross(position(b) - position(a), position(c) - position(a)), normal) < 0.0f) {
      std::swap(outward[i + 1], outward[i + 2]);
    }
  }
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, outward.size() * sizeof(unsigned int), outward.data(),
               GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        reinterpret_cast<void*>(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
  glBindVertexArray(0);
}

}  // namespace

glm::mat4 OrbitCamera::viewMatrix() const {
  glm::vec3 pos = position();
  return glm::lookAt(pos, target, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::mat4 OrbitCamera::projectionMatrix(float aspect) const {
  return glm::perspective(glm::radians(fovDegrees), aspect, 0.1f, 500.0f);
}

glm::vec3 OrbitCamera::position() const {
  float cp = std::cos(pitch);
  glm::vec3 offset{distance * cp * std::sin(yaw), distance * std::sin(pitch),
                   distance * cp * std::cos(yaw)};
  return target + offset;
}

bool Renderer::init(GLFWwindow* window, void* (*loader)(const char*)) {
  window_ = window;
  if (!gladLoadGLLoader(loader ? loader : reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
    return false;
  }

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glEnable(GL_MULTISAMPLE);

  if (!shader_.loadFromSource(shaders::surfaceVertex, shaders::surfaceFragment)) {
    return false;
  }
  if (!gridShader_.loadFromSource(kGridVertex, kGridFragment)) {
    return false;
  }
  if (!lineShader_.loadFromSource(kLineVertex, kLineFragment)) {
    return false;
  }

  if (!shadowShader_.loadFromSource(shaders::depthVertex, shaders::depthFragment) ||
      !createShadowMap())
    return false;
  lightMatrix_ = glm::ortho(-23.0f, 23.0f, -23.0f, 23.0f, 0.1f, 80.0f) *
                 glm::lookAt(glm::vec3(-20.0f, 34.0f, 16.0f), glm::vec3(0.0f), glm::vec3(0, 1, 0));
  createUnitMeshes();

  // Line dynamic buffer setup
  glGenVertexArrays(1, &lineVao_);
  glGenBuffers(1, &lineVbo_);
  glBindVertexArray(lineVao_);
  glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
  glBufferData(GL_ARRAY_BUFFER, 4096 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        reinterpret_cast<void*>(3 * sizeof(float)));
  glEnableVertexAttribArray(1);
  glBindVertexArray(0);

  return true;
}

void Renderer::shutdown() {
  for (auto* handle : {&cubeVao_, &sphereVao_, &gridVao_, &lineVao_}) {
    if (*handle) glDeleteVertexArrays(1, handle);
    *handle = 0;
  }
  for (auto* handle : {&cubeVbo_, &cubeEbo_, &sphereVbo_, &sphereEbo_, &gridVbo_, &lineVbo_}) {
    if (*handle) glDeleteBuffers(1, handle);
    *handle = 0;
  }
  if (shadowFramebuffer_) glDeleteFramebuffers(1, &shadowFramebuffer_);
  if (shadowTexture_) glDeleteTextures(1, &shadowTexture_);
  shadowFramebuffer_ = shadowTexture_ = 0;
  shader_.reset();
  shadowShader_.reset();
  gridShader_.reset();
  lineShader_.reset();
  trails_.clear();
}

void Renderer::createUnitMeshes() {
  const float cubeVerts[] = {
      -0.5f, -0.5f, -0.5f, 0,  0,  -1, 0.5f,  -0.5f, -0.5f, 0,  0,  -1,
      0.5f,  0.5f,  -0.5f, 0,  0,  -1, -0.5f, 0.5f,  -0.5f, 0,  0,  -1,
      -0.5f, -0.5f, 0.5f,  0,  0,  1,  0.5f,  -0.5f, 0.5f,  0,  0,  1,
      0.5f,  0.5f,  0.5f,  0,  0,  1,  -0.5f, 0.5f,  0.5f,  0,  0,  1,
      -0.5f, 0.5f,  0.5f,  -1, 0,  0,  -0.5f, 0.5f,  -0.5f, -1, 0,  0,
      -0.5f, -0.5f, -0.5f, -1, 0,  0,  -0.5f, -0.5f, 0.5f,  -1, 0,  0,
      0.5f,  0.5f,  0.5f,  1,  0,  0,  0.5f,  0.5f,  -0.5f, 1,  0,  0,
      0.5f,  -0.5f, -0.5f, 1,  0,  0,  0.5f,  -0.5f, 0.5f,  1,  0,  0,
      -0.5f, -0.5f, -0.5f, 0,  -1, 0,  0.5f,  -0.5f, -0.5f, 0,  -1, 0,
      0.5f,  -0.5f, 0.5f,  0,  -1, 0,  -0.5f, -0.5f, 0.5f,  0,  -1, 0,
      -0.5f, 0.5f,  -0.5f, 0,  1,  0,  0.5f,  0.5f,  -0.5f, 0,  1,  0,
      0.5f,  0.5f,  0.5f,  0,  1,  0,  -0.5f, 0.5f,  0.5f,  0,  1,  0,
  };
  const unsigned int cubeIdx[] = {0,  1,  2,  2,  3,  0,  4,  5,  6,  6,  7,  4,
                                  8,  9,  10, 10, 11, 8,  12, 13, 14, 14, 15, 12,
                                  16, 17, 18, 18, 19, 16, 20, 21, 22, 22, 23, 20};
  std::vector<float> cubeVec(std::begin(cubeVerts), std::end(cubeVerts));
  std::vector<unsigned int> cubeInd(std::begin(cubeIdx), std::end(cubeIdx));
  uploadMesh(cubeVao_, cubeVbo_, cubeEbo_, cubeVec, cubeInd, cubeIndexCount_);

  std::vector<float> sphereVerts;
  std::vector<unsigned int> sphereInd;
  const int stacks = 48;
  const int slices = 64;
  for (int i = 0; i <= stacks; ++i) {
    float v = static_cast<float>(i) / stacks;
    float phi = v * glm::pi<float>();
    for (int j = 0; j <= slices; ++j) {
      float u = static_cast<float>(j) / slices;
      float theta = u * (glm::pi<float>() * 2.0f);
      float x = std::cos(theta) * std::sin(phi);
      float y = std::cos(phi);
      float z = std::sin(theta) * std::sin(phi);
      sphereVerts.insert(sphereVerts.end(), {x * 0.5f, y * 0.5f, z * 0.5f, x, y, z});
    }
  }
  for (int i = 0; i < stacks; ++i) {
    for (int j = 0; j < slices; ++j) {
      unsigned int a = i * (slices + 1) + j;
      unsigned int b = (i + 1) * (slices + 1) + j;
      sphereInd.push_back(a);
      sphereInd.push_back(b);
      sphereInd.push_back(a + 1);
      sphereInd.push_back(a + 1);
      sphereInd.push_back(b);
      sphereInd.push_back(b + 1);
    }
  }
  uploadMesh(sphereVao_, sphereVbo_, sphereEbo_, sphereVerts, sphereInd, sphereIndexCount_);
}

void Renderer::onFramebufferResize(int width, int height) {
  framebufferWidth_ = width;
  framebufferHeight_ = height;
  glViewport(0, 0, width, height);
}

void Renderer::beginFrame(const OrbitCamera& camera, float aspect) {
  lastView_ = camera.viewMatrix();
  lastProjection_ = camera.projectionMatrix(aspect);
  lastCameraPos_ = camera.position();

  lineVertices_.clear();

  glClearColor(0.018f, 0.026f, 0.038f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  shader_.use();
  shader_.setMat4("uView", lastView_);
  shader_.setMat4("uProjection", lastProjection_);
  shader_.setMat4("uLightMatrix", lightMatrix_);
  shader_.setVec3("uCameraPos", lastCameraPos_);
}

void Renderer::drawGrid(float size, int divisions) {
  if (gridVao_ == 0) {
    std::vector<float> lines;
    const float half = size * 0.5f;
    const float step = size / divisions;
    for (int i = 0; i <= divisions; ++i) {
      const float x = -half + i * step;
      lines.insert(lines.end(), {x, 0.0f, -half, 0, 1, 0});
      lines.insert(lines.end(), {x, 0.0f, half, 0, 1, 0});
      lines.insert(lines.end(), {-half, 0.0f, x, 0, 1, 0});
      lines.insert(lines.end(), {half, 0.0f, x, 0, 1, 0});
    }
    gridVertexCount_ = static_cast<unsigned int>(lines.size() / 6);
    glGenVertexArrays(1, &gridVao_);
    glGenBuffers(1, &gridVbo_);
    glBindVertexArray(gridVao_);
    glBindBuffer(GL_ARRAY_BUFFER, gridVbo_);
    glBufferData(GL_ARRAY_BUFFER, lines.size() * sizeof(float), lines.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);
  }

  gridShader_.use();
  gridShader_.setMat4("uView", lastView_);
  gridShader_.setMat4("uProjection", lastProjection_);
  glBindVertexArray(gridVao_);
  glDrawArrays(GL_LINES, 0, gridVertexCount_);
  glBindVertexArray(0);

  shader_.use();
  shader_.setMat4("uView", lastView_);
  shader_.setMat4("uProjection", lastProjection_);
  shader_.setVec3("uCameraPos", lastCameraPos_);
}

bool Renderer::createShadowMap() {
  glGenFramebuffers(1, &shadowFramebuffer_);
  glGenTextures(1, &shadowTexture_);
  glBindTexture(GL_TEXTURE_2D, shadowTexture_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, 2048, 2048, 0, GL_DEPTH_COMPONENT, GL_FLOAT,
               nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
  const float border[] = {1.0f, 1.0f, 1.0f, 1.0f};
  glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, border);
  glBindFramebuffer(GL_FRAMEBUFFER, shadowFramebuffer_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowTexture_, 0);
  glDrawBuffer(GL_NONE);
  glReadBuffer(GL_NONE);
  const bool complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  return complete;
}

void Renderer::drawGeometry(const World::EntityRecord& rec, Shader& shader) {
  glm::mat4 model = rec.transform.matrix();
  unsigned int count;
  if (rec.collider.shape == ColliderShape::Sphere) {
    model = glm::scale(model, glm::vec3(rec.collider.radius * 2.0f));
    glBindVertexArray(sphereVao_);
    count = sphereIndexCount_;
  } else {
    model = glm::scale(model, rec.collider.halfExtents * 2.0f);
    glBindVertexArray(cubeVao_);
    count = cubeIndexCount_;
  }
  shader.setMat4("uModel", model);
  glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, nullptr);
}

void Renderer::drawWorld(const World& world, EntityId selectedEntity) {
  glViewport(0, 0, 2048, 2048);
  glBindFramebuffer(GL_FRAMEBUFFER, shadowFramebuffer_);
  glClear(GL_DEPTH_BUFFER_BIT);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(2.0f, 4.0f);
  shadowShader_.use();
  shadowShader_.setMat4("uLightMatrix", lightMatrix_);
  for (const auto& rec : world.entities())
    if (rec.active) drawGeometry(rec, shadowShader_);
  glDisable(GL_POLYGON_OFFSET_FILL);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glViewport(0, 0, framebufferWidth_, framebufferHeight_);

  shader_.use();
  shader_.setMat4("uView", lastView_);
  shader_.setMat4("uProjection", lastProjection_);
  shader_.setMat4("uLightMatrix", lightMatrix_);
  shader_.setVec3("uCameraPos", lastCameraPos_);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, shadowTexture_);
  shader_.setInt("uShadowMap", 0);
  for (const auto& rec : world.entities()) {
    if (!rec.active) continue;
    shader_.setVec3(
        "uColor", rec.entity.id == selectedEntity ? glm::vec3(0.2f, 0.8f, 0.9f) : rec.render.color);
    shader_.setFloat("uMetallic", rec.render.metallic);
    shader_.setFloat("uRoughness", rec.render.roughness);
    shader_.setInt("uSurface", rec.render.surface);
    drawGeometry(rec, shader_);
  }
  glBindVertexArray(0);
}

void Renderer::drawTrails(const World& world) {
  for (auto it = trails_.begin(); it != trails_.end();) {
    if (!world.alive(Entity{it->first}))
      it = trails_.erase(it);
    else
      ++it;
  }
  for (const auto& rec : world.entities()) {
    if (!rec.active || rec.rigidBody.isStatic || rec.collider.shape != ColliderShape::Sphere)
      continue;
    if (trails_.find(rec.entity.id) == trails_.end() && trails_.size() >= 64) continue;
    auto& path = trails_[rec.entity.id];
    if (path.empty() || glm::length(rec.transform.position - path.back()) > 0.025f) {
      path.push_back(rec.transform.position);
      if (path.size() > 72) path.pop_front();
    }
    for (std::size_t i = 1; i < path.size(); ++i) {
      const float fade = static_cast<float>(i) / path.size();
      drawLine(path[i - 1], path[i], rec.render.color * (0.25f + 0.75f * fade));
    }
  }
}

void Renderer::drawLine(const glm::vec3& start, const glm::vec3& end, const glm::vec3& color) {
  if (lineVertices_.size() >= 1200000) return;
  lineVertices_.insert(lineVertices_.end(), {start.x, start.y, start.z, color.r, color.g, color.b});
  lineVertices_.insert(lineVertices_.end(), {end.x, end.y, end.z, color.r, color.g, color.b});
}

void Renderer::drawContacts(const std::vector<ContactManifold>& contacts) {
  for (const auto& c : contacts) {
    glm::vec3 normalColor(1.0f, 0.25f, 0.2f);
    drawLine(c.point, c.point + c.normal * 0.45f, normalColor);
    // Draw cross at contact point
    float s = 0.06f;
    drawLine(c.point - glm::vec3(s, 0, 0), c.point + glm::vec3(s, 0, 0),
             glm::vec3(1.0f, 0.9f, 0.1f));
    drawLine(c.point - glm::vec3(0, s, 0), c.point + glm::vec3(0, s, 0),
             glm::vec3(1.0f, 0.9f, 0.1f));
    drawLine(c.point - glm::vec3(0, 0, s), c.point + glm::vec3(0, 0, s),
             glm::vec3(1.0f, 0.9f, 0.1f));
  }
}

void Renderer::drawJoints(const World& world, const std::vector<DistanceJoint>& distanceJoints,
                          const std::vector<SpringJoint>& springJoints,
                          const MouseSpring& mouseSpring) {
  glm::vec3 jointColor(0.45f, 0.43f, 0.36f);
  for (const auto& j : distanceJoints) {
    const auto* tA = world.transform(Entity{j.entityA});
    const auto* tB = world.transform(Entity{j.entityB});
    if (tA && tB) {
      glm::vec3 pA = tA->position + tA->rotation * j.localAnchorA;
      glm::vec3 pB = tB->position + tB->rotation * j.localAnchorB;
      drawLine(pA, pB, jointColor);
    }
  }

  glm::vec3 springColor(1.0f, 0.6f, 0.1f);
  for (const auto& s : springJoints) {
    const auto* tA = world.transform(Entity{s.entityA});
    const auto* tB = world.transform(Entity{s.entityB});
    if (tA && tB) {
      glm::vec3 pA = tA->position + tA->rotation * s.localAnchorA;
      glm::vec3 pB = tB->position + tB->rotation * s.localAnchorB;
      drawLine(pA, pB, springColor);
    }
  }

  if (mouseSpring.active && mouseSpring.entity != kInvalidEntity) {
    const auto* t = world.transform(Entity{mouseSpring.entity});
    if (t) {
      glm::vec3 anchorWorld = t->position + t->rotation * mouseSpring.localAnchor;
      drawLine(anchorWorld, mouseSpring.worldTarget, glm::vec3(0.3f, 1.0f, 0.4f));
    }
  }
}

void Renderer::drawSpatialHashCells(const SpatialHash& spatialHash, float cellSize) {
  glm::vec3 gridColor(0.25f, 0.35f, 0.45f);
  std::size_t drawn = 0;
  for (const auto& [cell, ids] : spatialHash.grid()) {
    if (++drawn > 1024) break;
    if (ids.empty()) continue;
    glm::vec3 minP = glm::vec3(cell.x, cell.y, cell.z) * cellSize;
    glm::vec3 maxP = minP + glm::vec3(cellSize);

    // Draw wireframe box
    drawLine(glm::vec3(minP.x, minP.y, minP.z), glm::vec3(maxP.x, minP.y, minP.z), gridColor);
    drawLine(glm::vec3(maxP.x, minP.y, minP.z), glm::vec3(maxP.x, minP.y, maxP.z), gridColor);
    drawLine(glm::vec3(maxP.x, minP.y, maxP.z), glm::vec3(minP.x, minP.y, maxP.z), gridColor);
    drawLine(glm::vec3(minP.x, minP.y, maxP.z), glm::vec3(minP.x, minP.y, minP.z), gridColor);

    drawLine(glm::vec3(minP.x, maxP.y, minP.z), glm::vec3(maxP.x, maxP.y, minP.z), gridColor);
    drawLine(glm::vec3(maxP.x, maxP.y, minP.z), glm::vec3(maxP.x, maxP.y, maxP.z), gridColor);
    drawLine(glm::vec3(maxP.x, maxP.y, maxP.z), glm::vec3(minP.x, maxP.y, maxP.z), gridColor);
    drawLine(glm::vec3(minP.x, maxP.y, maxP.z), glm::vec3(minP.x, maxP.y, minP.z), gridColor);

    drawLine(glm::vec3(minP.x, minP.y, minP.z), glm::vec3(minP.x, maxP.y, minP.z), gridColor);
    drawLine(glm::vec3(maxP.x, minP.y, minP.z), glm::vec3(maxP.x, maxP.y, minP.z), gridColor);
    drawLine(glm::vec3(maxP.x, minP.y, maxP.z), glm::vec3(maxP.x, maxP.y, maxP.z), gridColor);
    drawLine(glm::vec3(minP.x, minP.y, maxP.z), glm::vec3(minP.x, maxP.y, maxP.z), gridColor);
  }
}

void Renderer::flushLines() {
  if (lineVertices_.empty()) return;

  lineShader_.use();
  lineShader_.setMat4("uView", lastView_);
  lineShader_.setMat4("uProjection", lastProjection_);

  glBindVertexArray(lineVao_);
  glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
  glBufferData(GL_ARRAY_BUFFER, lineVertices_.size() * sizeof(float), lineVertices_.data(),
               GL_DYNAMIC_DRAW);
  glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lineVertices_.size() / 6));
  glBindVertexArray(0);
}

Ray Renderer::screenPointToRay(double mouseX, double mouseY, int screenWidth, int screenHeight,
                               const OrbitCamera& camera) const {
  float x = (2.0f * static_cast<float>(mouseX)) / static_cast<float>(screenWidth) - 1.0f;
  float y = 1.0f - (2.0f * static_cast<float>(mouseY)) / static_cast<float>(screenHeight);

  glm::mat4 invProj = glm::inverse(
      camera.projectionMatrix(static_cast<float>(screenWidth) / std::max(screenHeight, 1)));
  glm::mat4 invView = glm::inverse(camera.viewMatrix());

  glm::vec4 rayClip = glm::vec4(x, y, -1.0f, 1.0f);
  glm::vec4 rayEye = invProj * rayClip;
  rayEye = glm::vec4(rayEye.x, rayEye.y, -1.0f, 0.0f);

  glm::vec3 rayWorld = glm::normalize(glm::vec3(invView * rayEye));

  Ray ray;
  ray.origin = camera.position();
  ray.direction = rayWorld;
  return ray;
}

void Renderer::endFrame() { glfwSwapBuffers(window_); }

}  // namespace simforge
