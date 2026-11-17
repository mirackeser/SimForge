#include <exception>
#include <iostream>

#include "app/Application.hpp"

int main() {
  try {
    simforge::Application app;
    return app.run();
  } catch (const std::exception& error) {
    std::cerr << "SimForge: " << error.what() << '\n';
    return 1;
  }
}
