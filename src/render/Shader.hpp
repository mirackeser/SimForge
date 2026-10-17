#pragma once

#include <glm/glm.hpp>
#include <string>

namespace simforge {

class Shader {
 public:
  Shader() = default;
  ~Shader();

  Shader(const Shader&) = delete;
  Shader& operator=(const Shader&) = delete;

  bool loadFromSource(const std::string& vertexSrc, const std::string& fragmentSrc);
  void use() const;

  void setMat4(const std::string& name, const glm::mat4& value) const;
  void setVec3(const std::string& name, const glm::vec3& value) const;
  void setFloat(const std::string& name, float value) const;

  unsigned int id() const { return programId_; }

 private:
  unsigned int programId_{0};
};

}  // namespace simforge
