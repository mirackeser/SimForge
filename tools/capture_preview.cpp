#include <GLFW/glfw3.h>

#include <exception>
#include <iostream>

#include "app/Application.hpp"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: SimForgePreview output.ppm\n";
    return 1;
  }
  if (!glfwInit()) return 1;
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
  GLFWwindow* window = glfwCreateWindow(1600, 900, "SimForge Preview", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  int result = 1;
  {
    // Application owns GL resources; release them while this context is current.
    simforge::Application app;
    try {
      result =
          app.capturePreview(argv[1], reinterpret_cast<void* (*)(const char*)>(glfwGetProcAddress))
              ? 0
              : 1;
    } catch (const std::exception& error) {
      std::cerr << "Preview capture: " << error.what() << '\n';
    }
    // Application shutdown also terminates GLFW and destroys its platform state.
  }
  return result;
}
