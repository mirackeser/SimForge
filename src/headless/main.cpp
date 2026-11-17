#include "scene/DemoScenes.hpp"

#include <charconv>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

simforge::DemoScene parseScene(const std::string& name) {
  using simforge::DemoScene;
  if (name == "domino") return DemoScene::DominoStack;
  if (name == "cradle") return DemoScene::NewtonsCradle;
  if (name == "jenga") return DemoScene::JengaTower;
  if (name == "bridge") return DemoScene::RopeBridge;
  if (name == "rain") return DemoScene::ParticleRain;
  if (name == "stress") return DemoScene::MixedStress;
  throw std::invalid_argument("Unknown scene: " + name);
}

int parseSteps(const std::string& value) {
  int steps = 0;
  auto result = std::from_chars(value.data(), value.data() + value.size(), steps);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size() || steps < 1 || steps > 1000000) {
    throw std::invalid_argument("Steps must be an integer in [1, 1000000]");
  }
  return steps;
}

bool finite(const glm::vec3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    auto scene = simforge::DemoScene::DominoStack;
    int steps = 120;
    std::string csvPath;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << "SimForgeHeadless [--scene domino|cradle|jenga|bridge|rain|stress] "
                     "[--steps 1..1000000] [--csv path]\n";
        return 0;
      }
      if (arg != "--scene" && arg != "--steps" && arg != "--csv") {
        throw std::invalid_argument("Unknown option: " + arg);
      }
      if (++i == argc) throw std::invalid_argument("Missing value for " + arg);
      if (arg == "--scene") scene = parseScene(argv[i]);
      if (arg == "--steps") steps = parseSteps(argv[i]);
      if (arg == "--csv") csvPath = argv[i];
    }

    std::ofstream csv;
    if (!csvPath.empty()) {
      csv.open(csvPath);
      if (!csv) throw std::runtime_error("Cannot open CSV: " + csvPath);
      csv << "step,time,entity,x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz\n";
      csv << std::setprecision(9);
    }
    simforge::World world;
    simforge::PhysicsWorld physics;
    simforge::loadDemoScene(world, physics, scene);
    for (int i = 1; i <= steps; ++i) {
      physics.stepFixed(world);
      for (const auto& rec : world.entities()) {
        if (!rec.active) continue;
        const auto& p = rec.transform.position;
        const auto& v = rec.rigidBody.velocity;
        const auto& q = rec.transform.rotation;
        const auto& w = rec.rigidBody.angularVelocity;
        if (!finite(p) || !finite(v) || !finite(w) || !std::isfinite(q.w) || !std::isfinite(q.x) ||
            !std::isfinite(q.y) || !std::isfinite(q.z)) {
          throw std::runtime_error("Non-finite state at step " + std::to_string(i) +
                                   ", entity " + std::to_string(rec.entity.id));
        }
        if (csv.is_open()) {
          csv << i << ',' << physics.simulationTime() << ',' << rec.entity.id << ','
              << p.x << ',' << p.y << ',' << p.z << ',' << v.x << ',' << v.y << ',' << v.z << ','
              << q.w << ',' << q.x << ',' << q.y << ',' << q.z << ',' << w.x << ',' << w.y << ',' << w.z << '\n';
        }
      }
    }
    if (csv.is_open()) {
      csv.flush();
      if (!csv) throw std::runtime_error("Failed to write CSV: " + csvPath);
    }
    std::cout << "scene=\"" << simforge::demoSceneName(scene) << "\" steps=" << steps
              << " time=" << physics.simulationTime() << " entities=" << world.entities().size()
              << " contacts=" << physics.collisionCountLastFrame() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "SimForgeHeadless: " << error.what() << '\n';
    return 1;
  }
}
