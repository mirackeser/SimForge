#include "scene/DemoScenes.hpp"

#include <glm/gtc/quaternion.hpp>
#include <random>

namespace simforge {

namespace {

Entity spawnBox(World& world, const glm::vec3& pos, const glm::vec3& halfExtents, float mass, bool staticBody,
                const glm::vec3& color, const glm::quat& rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
  Entity e = world.createEntity("box");
  auto* t = world.transform(e);
  auto* rb = world.rigidBody(e);
  auto* c = world.collider(e);
  auto* r = world.render(e);

  t->position = pos;
  t->rotation = rot;
  c->shape = ColliderShape::Box;
  c->halfExtents = halfExtents;

  rb->isStatic = staticBody;
  rb->inverseMass = staticBody ? 0.0f : 1.0f / mass;
  rb->restitution = 0.25f;
  rb->staticFriction = 0.6f;
  rb->dynamicFriction = 0.4f;
  rb->computeInertia(ColliderShape::Box, 0.0f, halfExtents);

  r->color = color;
  return e;
}

Entity spawnSphere(World& world, const glm::vec3& pos, float radius, float mass, const glm::vec3& color,
                   bool staticBody = false) {
  Entity e = world.createEntity("sphere");
  auto* t = world.transform(e);
  auto* rb = world.rigidBody(e);
  auto* c = world.collider(e);
  auto* r = world.render(e);

  t->position = pos;
  c->shape = ColliderShape::Sphere;
  c->radius = radius;

  rb->isStatic = staticBody;
  rb->inverseMass = staticBody ? 0.0f : 1.0f / mass;
  rb->restitution = 0.5f;
  rb->staticFriction = 0.4f;
  rb->dynamicFriction = 0.3f;
  rb->computeInertia(ColliderShape::Sphere, radius, glm::vec3(0.0f));

  r->color = color;
  return e;
}

void buildGround(World& world) {
  spawnBox(world, glm::vec3(0.0f, -0.5f, 0.0f), glm::vec3(30.0f, 0.5f, 30.0f), 0.0f, true,
           glm::vec3(0.24f, 0.28f, 0.33f));
}

}  // namespace

const char* demoSceneName(DemoScene scene) {
  switch (scene) {
    case DemoScene::DominoStack:
      return "Domino Chain & Towers";
    case DemoScene::NewtonsCradle:
      return "Newton's Cradle (Constraints)";
    case DemoScene::JengaTower:
      return "Jenga Stacking Tower";
    case DemoScene::RopeBridge:
      return "Suspended Rope Bridge";
    case DemoScene::ParticleRain:
      return "Particle Rain (Broadphase)";
    case DemoScene::MixedStress:
      return "Mixed Stress Sandbox";
  }
  return "Unknown";
}

void loadDemoScene(World& world, PhysicsWorld& physics, DemoScene scene) {
  world.clear();
  physics.reset();
  buildGround(world);

  switch (scene) {
    case DemoScene::DominoStack: {
      // Domino line
      const int dominoCount = 14;
      for (int i = 0; i < dominoCount; ++i) {
        spawnBox(world, glm::vec3(-5.0f + i * 0.75f, 0.6f, 0.0f), glm::vec3(0.12f, 0.6f, 0.28f), 1.5f,
                 false, glm::vec3(0.92f, 0.38f, 0.22f));
      }

      // Domino pyramid target
      for (int layer = 0; layer < 5; ++layer) {
        for (int j = 0; j <= layer; ++j) {
          spawnBox(world, glm::vec3(6.5f + j * 0.6f, 0.35f + layer * 0.55f, -1.0f + layer * 0.25f),
                   glm::vec3(0.45f, 0.3f, 0.45f), 1.2f, false, glm::vec3(0.28f, 0.68f, 0.95f));
        }
      }

      // Initial pusher sphere
      Entity pusher = spawnSphere(world, glm::vec3(-7.5f, 1.2f, 0.0f), 0.6f, 5.0f, glm::vec3(0.98f, 0.85f, 0.15f));
      auto* rbPusher = world.rigidBody(pusher);
      if (rbPusher) {
        rbPusher->velocity = glm::vec3(4.5f, 0.0f, 0.0f);
      }
      break;
    }

    case DemoScene::NewtonsCradle: {
      // Top suspension beam (static)
      Entity beam = spawnBox(world, glm::vec3(0.0f, 8.0f, 0.0f), glm::vec3(4.0f, 0.2f, 0.5f), 0.0f, true,
                             glm::vec3(0.4f, 0.45f, 0.5f));

      const int ballCount = 5;
      const float radius = 0.45f;
      const float cableLength = 5.0f;
      const float spacing = radius * 2.02f;
      const float startX = -((ballCount - 1) * spacing) * 0.5f;

      for (int i = 0; i < ballCount; ++i) {
        float x = startX + i * spacing;
        float y = 8.0f - cableLength;
        glm::vec3 ballPos(x, y, 0.0f);

        // Displace the leftmost ball upwards and outwards to start the pendulum swing
        if (i == 0) {
          ballPos = glm::vec3(x - 3.2f, y + 1.8f, 0.0f);
        }

        Entity ball = spawnSphere(world, ballPos, radius, 3.0f, glm::vec3(0.85f, 0.88f, 0.92f));
        auto* rbBall = world.rigidBody(ball);
        if (rbBall) {
          rbBall->restitution = 0.98f;  // Elastic collision
          rbBall->linearDamping = 0.001f;
        }

        DistanceJoint joint;
        joint.entityA = beam.id;
        joint.entityB = ball.id;
        joint.localAnchorA = glm::vec3(x, -0.2f, 0.0f);
        joint.localAnchorB = glm::vec3(0.0f, 0.0f, 0.0f);
        joint.targetDistance = cableLength;
        joint.stiffness = 1.0f;
        physics.addDistanceJoint(joint);
      }
      break;
    }

    case DemoScene::JengaTower: {
      const int layers = 10;
      const float blockWidth = 0.55f;
      const float blockHeight = 0.35f;
      const float blockLength = 1.75f;
      const float blockMass = 1.0f;

      for (int layer = 0; layer < layers; ++layer) {
        float y = blockHeight * 0.5f + layer * blockHeight;
        bool isXAligned = (layer % 2 == 0);

        for (int i = -1; i <= 1; ++i) {
          glm::vec3 pos;
          glm::vec3 halfExtents;

          if (isXAligned) {
            pos = glm::vec3(0.0f, y, i * (blockWidth + 0.02f));
            halfExtents = glm::vec3(blockLength * 0.5f, blockHeight * 0.5f, blockWidth * 0.5f);
          } else {
            pos = glm::vec3(i * (blockWidth + 0.02f), y, 0.0f);
            halfExtents = glm::vec3(blockWidth * 0.5f, blockHeight * 0.5f, blockLength * 0.5f);
          }

          glm::vec3 color = isXAligned ? glm::vec3(0.88f, 0.65f, 0.42f) : glm::vec3(0.82f, 0.58f, 0.35f);
          Entity e = spawnBox(world, pos, halfExtents, blockMass, false, color);
          auto* rb = world.rigidBody(e);
          if (rb) {
            rb->staticFriction = 0.75f;
            rb->dynamicFriction = 0.55f;
            rb->restitution = 0.08f;
          }
        }
      }

      // Wrecking ball coming from side to test collapse
      Entity ball = spawnSphere(world, glm::vec3(-6.0f, 2.5f, 0.0f), 0.65f, 6.0f, glm::vec3(0.95f, 0.3f, 0.25f));
      auto* rbWreck = world.rigidBody(ball);
      if (rbWreck) {
        rbWreck->velocity = glm::vec3(5.0f, 0.0f, 0.0f);
      }
      break;
    }

    case DemoScene::RopeBridge: {
      // Left and right anchor pillars
      Entity towerL = spawnBox(world, glm::vec3(-7.0f, 2.5f, 0.0f), glm::vec3(0.8f, 2.5f, 1.5f), 0.0f, true,
                               glm::vec3(0.45f, 0.48f, 0.52f));
      Entity towerR = spawnBox(world, glm::vec3(7.0f, 2.5f, 0.0f), glm::vec3(0.8f, 2.5f, 1.5f), 0.0f, true,
                               glm::vec3(0.45f, 0.48f, 0.52f));

      const int plankCount = 10;
      const float plankSpacing = 1.25f;
      Entity prevEntity = towerL;
      glm::vec3 prevAnchor(0.8f, 2.3f, 0.0f);

      for (int i = 0; i < plankCount; ++i) {
        float t = (i + 0.5f) / plankCount;
        float x = -6.0f + i * plankSpacing;
        float sag = std::sin(t * 3.14159f) * 1.5f;
        float y = 4.8f - sag;

        Entity plank = spawnBox(world, glm::vec3(x, y, 0.0f), glm::vec3(0.55f, 0.12f, 1.2f), 1.5f, false,
                                glm::vec3(0.78f, 0.56f, 0.35f));

        DistanceJoint j;
        j.entityA = prevEntity.id;
        j.entityB = plank.id;
        j.localAnchorA = prevAnchor;
        j.localAnchorB = glm::vec3(-0.55f, 0.0f, 0.0f);
        j.targetDistance = 0.25f;
        j.stiffness = 1.0f;
        physics.addDistanceJoint(j);

        prevEntity = plank;
        prevAnchor = glm::vec3(0.55f, 0.0f, 0.0f);
      }

      // Connect last plank to right tower
      DistanceJoint jEnd;
      jEnd.entityA = prevEntity.id;
      jEnd.entityB = towerR.id;
      jEnd.localAnchorA = prevAnchor;
      jEnd.localAnchorB = glm::vec3(-0.8f, 2.3f, 0.0f);
      jEnd.targetDistance = 0.25f;
      physics.addDistanceJoint(jEnd);

      // Spheres rolling across bridge
      for (int k = 0; k < 4; ++k) {
        spawnSphere(world, glm::vec3(-5.0f + k * 2.5f, 6.0f + k * 0.8f, 0.0f), 0.35f, 2.0f,
                    glm::vec3(0.9f, 0.4f, 0.2f));
      }
      break;
    }

    case DemoScene::ParticleRain: {
      std::mt19937 rng{42};
      std::uniform_real_distribution<float> xz(-10.0f, 10.0f);
      std::uniform_real_distribution<float> y(8.0f, 18.0f);
      std::uniform_real_distribution<float> radius(0.18f, 0.38f);
      for (int i = 0; i < 180; ++i) {
        float r = radius(rng);
        spawnSphere(world, glm::vec3(xz(rng), y(rng), xz(rng)), r, r * r * r * 8.0f,
                    glm::vec3(0.35f + r, 0.55f, 0.95f - r));
      }
      break;
    }

    case DemoScene::MixedStress: {
      // 36 dynamic boxes
      for (int x = 0; x < 6; ++x) {
        for (int z = 0; z < 6; ++z) {
          spawnBox(world, glm::vec3(-5.0f + x * 2.0f, 0.6f, -5.0f + z * 2.0f), glm::vec3(0.7f, 0.5f, 0.7f),
                   3.0f, false, glm::vec3(0.55f, 0.6f, 0.68f));
        }
      }
      // 80 dynamic bouncing spheres
      for (int i = 0; i < 80; ++i) {
        spawnSphere(world, glm::vec3(-6.0f + (i % 10) * 1.3f, 4.5f + (i / 10) * 1.2f, 4.0f), 0.35f, 1.2f,
                    glm::vec3(0.95f, 0.45f, 0.25f));
      }
      break;
    }
  }
}

}  // namespace simforge
