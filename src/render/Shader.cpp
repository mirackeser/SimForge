#include "render/Shader.hpp"

#include <glad/glad.h>

#include <iostream>
#include <vector>

namespace simforge {

namespace {

unsigned int compileShader(unsigned int type, const char* source) {
  unsigned int shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  int success = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    char log[1024];
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    std::cerr << "Shader compile error: " << log << '\n';
    glDeleteShader(shader);
    return 0;
  }
  return shader;
}

}  // namespace

Shader::~Shader() { reset(); }

void Shader::reset() {
  if (programId_ != 0) glDeleteProgram(programId_);
  programId_ = 0;
}

bool Shader::loadFromSource(const std::string& vertexSrc, const std::string& fragmentSrc) {
  unsigned int vs = compileShader(GL_VERTEX_SHADER, vertexSrc.c_str());
  unsigned int fs = compileShader(GL_FRAGMENT_SHADER, fragmentSrc.c_str());
  if (vs == 0 || fs == 0) {
    if (vs) glDeleteShader(vs);
    if (fs) glDeleteShader(fs);
    return false;
  }
  const unsigned int replacement = glCreateProgram();
  glAttachShader(replacement, vs);
  glAttachShader(replacement, fs);
  glLinkProgram(replacement);
  glDeleteShader(vs);
  glDeleteShader(fs);

  int success = 0;
  glGetProgramiv(replacement, GL_LINK_STATUS, &success);
  if (!success) {
    char log[1024];
    glGetProgramInfoLog(replacement, sizeof(log), nullptr, log);
    std::cerr << "Program link error: " << log << '\n';
    glDeleteProgram(replacement);
    return false;
  }
  reset();
  programId_ = replacement;
  return true;
}

void Shader::setInt(const std::string& name, int value) const {
  glUniform1i(glGetUniformLocation(programId_, name.c_str()), value);
}

void Shader::use() const { glUseProgram(programId_); }

void Shader::setMat4(const std::string& name, const glm::mat4& value) const {
  glUniformMatrix4fv(glGetUniformLocation(programId_, name.c_str()), 1, GL_FALSE, &value[0][0]);
}

void Shader::setVec3(const std::string& name, const glm::vec3& value) const {
  glUniform3fv(glGetUniformLocation(programId_, name.c_str()), 1, &value[0]);
}

void Shader::setFloat(const std::string& name, float value) const {
  glUniform1f(glGetUniformLocation(programId_, name.c_str()), value);
}

}  // namespace simforge
