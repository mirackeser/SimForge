#include "physics/Collision.hpp"

#include <glm/gtc/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace simforge {

namespace {

// Clamps point to box half-extents in local coordinate space
glm::vec3 closestPointOnLocalBox(const glm::vec3& localPoint, const glm::vec3& halfExtents) {
  return glm::clamp(localPoint, -halfExtents, halfExtents);
}

// Support point on an OBB in world space for a given direction vector
glm::vec3 getOBBSupportPoint(const glm::vec3& center, const glm::mat3& R, const glm::vec3& halfExtents,
                             const glm::vec3& dir) {
  glm::vec3 result = center;
  for (int i = 0; i < 3; ++i) {
    glm::vec3 axis = R[i];
    float sign = (glm::dot(axis, dir) >= 0.0f) ? 1.0f : -1.0f;
    result += axis * (halfExtents[i] * sign);
  }
  return result;
}

}  // namespace

std::optional<ContactManifold> collideSphereSphere(const TransformComponent& a, const ColliderComponent& ca,
                                                   const TransformComponent& b, const ColliderComponent& cb) {
  glm::vec3 delta = b.position - a.position;
  float distSq = glm::dot(delta, delta);
  float radiusSum = ca.radius + cb.radius;
  if (distSq >= radiusSum * radiusSum) {
    return std::nullopt;
  }

  float dist = std::sqrt(distSq);
  ContactManifold m;
  m.normal = dist > 1e-5f ? delta / dist : glm::vec3(0.0f, 1.0f, 0.0f);
  m.penetration = radiusSum - dist;
  m.point = a.position + m.normal * (ca.radius - m.penetration * 0.5f);
  return m;
}

std::optional<ContactManifold> collideSphereBox(const TransformComponent& sphere, const ColliderComponent& cs,
                                                const TransformComponent& box, const ColliderComponent& cb) {
  glm::mat3 boxRot = glm::mat3_cast(box.rotation);
  glm::mat3 invBoxRot = glm::transpose(boxRot);

  // Transform sphere center into box local coordinate frame
  glm::vec3 localSpherePos = invBoxRot * (sphere.position - box.position);
  glm::vec3 localClosest = closestPointOnLocalBox(localSpherePos, cb.halfExtents);
  glm::vec3 localDelta = localSpherePos - localClosest;
  float distSq = glm::dot(localDelta, localDelta);

  if (distSq > cs.radius * cs.radius) {
    return std::nullopt;
  }

  float dist = std::sqrt(distSq);
  ContactManifold m;

  if (dist > 1e-5f) {
    glm::vec3 localNormal = localDelta / dist;
    m.normal = -(boxRot * localNormal);
    m.penetration = cs.radius - dist;
    m.point = box.position + boxRot * localClosest;
  } else {
    // Sphere center is inside box: find closest face
    glm::vec3 absPos = glm::abs(localSpherePos);
    glm::vec3 pen = cb.halfExtents - absPos;

    glm::vec3 localNormal(0.0f);
    if (pen.x < pen.y && pen.x < pen.z) {
      localNormal = glm::vec3(localSpherePos.x >= 0.0f ? 1.0f : -1.0f, 0.0f, 0.0f);
      m.penetration = cs.radius + pen.x;
    } else if (pen.y < pen.z) {
      localNormal = glm::vec3(0.0f, localSpherePos.y >= 0.0f ? 1.0f : -1.0f, 0.0f);
      m.penetration = cs.radius + pen.y;
    } else {
      localNormal = glm::vec3(0.0f, 0.0f, localSpherePos.z >= 0.0f ? 1.0f : -1.0f);
      m.penetration = cs.radius + pen.z;
    }

    m.normal = -(boxRot * localNormal);
    m.point = box.position + boxRot * (localSpherePos + localNormal * glm::dot(pen, glm::abs(localNormal)));
  }

  return m;
}

std::optional<ContactManifold> collideBoxBox(const TransformComponent& a, const ColliderComponent& ca,
                                             const TransformComponent& b, const ColliderComponent& cb) {
  glm::mat3 RA = glm::mat3_cast(a.rotation);
  glm::mat3 RB = glm::mat3_cast(b.rotation);

  glm::vec3 D = b.position - a.position;

  // 15 candidate separating axes:
  // 3 axes of A, 3 axes of B, 9 cross products
  glm::vec3 axes[15];
  axes[0] = RA[0];
  axes[1] = RA[1];
  axes[2] = RA[2];
  axes[3] = RB[0];
  axes[4] = RB[1];
  axes[5] = RB[2];

  int axisCount = 6;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      glm::vec3 crossAxis = glm::cross(RA[i], RB[j]);
      float lenSq = glm::dot(crossAxis, crossAxis);
      if (lenSq > 1e-5f) {
        axes[axisCount++] = crossAxis / std::sqrt(lenSq);
      }
    }
  }

  float minOverlap = std::numeric_limits<float>::max();
  glm::vec3 bestAxis(0.0f, 1.0f, 0.0f);

  for (int i = 0; i < axisCount; ++i) {
    glm::vec3 L = axes[i];
    float rA = ca.halfExtents.x * std::abs(glm::dot(RA[0], L)) +
               ca.halfExtents.y * std::abs(glm::dot(RA[1], L)) +
               ca.halfExtents.z * std::abs(glm::dot(RA[2], L));

    float rB = cb.halfExtents.x * std::abs(glm::dot(RB[0], L)) +
               cb.halfExtents.y * std::abs(glm::dot(RB[1], L)) +
               cb.halfExtents.z * std::abs(glm::dot(RB[2], L));

    float dist = std::abs(glm::dot(D, L));
    float overlap = (rA + rB) - dist;

    if (overlap <= 0.0f) {
      return std::nullopt;  // Separating axis found
    }

    if (overlap < minOverlap) {
      minOverlap = overlap;
      bestAxis = L;
    }
  }

  // Ensure normal points from A to B
  if (glm::dot(bestAxis, D) < 0.0f) {
    bestAxis = -bestAxis;
  }

  ContactManifold m;
  m.normal = bestAxis;
  m.penetration = minOverlap;

  // Contact point: midpoint between support points of A and B
  glm::vec3 ptA = getOBBSupportPoint(a.position, RA, ca.halfExtents, bestAxis);
  glm::vec3 ptB = getOBBSupportPoint(b.position, RB, cb.halfExtents, -bestAxis);
  m.point = (ptA + ptB) * 0.5f;

  return m;
}

void resolveContact(RigidBodyComponent& bodyA, TransformComponent& transformA,
                    RigidBodyComponent& bodyB, TransformComponent& transformB,
                    const ContactManifold& contact, bool enableFriction, bool enableRotation) {
  if (bodyA.isStatic && bodyB.isStatic) return;

  const float invMassA = bodyA.isStatic ? 0.0f : bodyA.inverseMass;
  const float invMassB = bodyB.isStatic ? 0.0f : bodyB.inverseMass;
  glm::vec3 normal = contact.normal;
  glm::vec3 rA = contact.point - transformA.position;
  glm::vec3 rB = contact.point - transformB.position;

  // Point velocities including angular velocity: v_p = v + w x r
  glm::vec3 vA_p = bodyA.velocity + ((bodyA.isStatic || !enableRotation) ? glm::vec3(0.0f) : glm::cross(bodyA.angularVelocity, rA));
  glm::vec3 vB_p = bodyB.velocity + ((bodyB.isStatic || !enableRotation) ? glm::vec3(0.0f) : glm::cross(bodyB.angularVelocity, rB));
  glm::vec3 relativeVelocity = vB_p - vA_p;

  float velAlongNormal = glm::dot(relativeVelocity, normal);
  if (velAlongNormal > 0.0f) return;  // Objects separating

  // Angular mass terms along normal: (I^-1 (r x n)) x r
  glm::vec3 angA_n = (bodyA.isStatic || !enableRotation) ? glm::vec3(0.0f)
                                    : glm::cross(bodyA.inverseInertiaTensorWorld * glm::cross(rA, normal), rA);
  glm::vec3 angB_n = (bodyB.isStatic || !enableRotation) ? glm::vec3(0.0f)
                                    : glm::cross(bodyB.inverseInertiaTensorWorld * glm::cross(rB, normal), rB);

  float invMassNormal = invMassA + invMassB + glm::dot(normal, angA_n + angB_n);
  if (invMassNormal <= 0.0f) return;

  float restitution = std::min(bodyA.restitution, bodyB.restitution);
  float jn = -(1.0f + restitution) * velAlongNormal / invMassNormal;
  jn = std::max(jn, 0.0f);

  glm::vec3 normalImpulse = jn * normal;

  // Apply normal impulse
  if (!bodyA.isStatic) {
    bodyA.velocity -= normalImpulse * bodyA.inverseMass;
    if (enableRotation) bodyA.angularVelocity -= bodyA.inverseInertiaTensorWorld * glm::cross(rA, normalImpulse);
  }
  if (!bodyB.isStatic) {
    bodyB.velocity += normalImpulse * bodyB.inverseMass;
    if (enableRotation) bodyB.angularVelocity += bodyB.inverseInertiaTensorWorld * glm::cross(rB, normalImpulse);
  }

  if (!enableFriction) return;

  // Coulomb Friction Model:
  // Recompute relative velocity at contact point after normal impulse
  vA_p = bodyA.velocity + ((bodyA.isStatic || !enableRotation) ? glm::vec3(0.0f) : glm::cross(bodyA.angularVelocity, rA));
  vB_p = bodyB.velocity + ((bodyB.isStatic || !enableRotation) ? glm::vec3(0.0f) : glm::cross(bodyB.angularVelocity, rB));
  relativeVelocity = vB_p - vA_p;

  glm::vec3 tangent = relativeVelocity - glm::dot(relativeVelocity, normal) * normal;
  float tangentLen = glm::length(tangent);
  if (tangentLen > 1e-5f) {
    tangent /= tangentLen;
    glm::vec3 angA_t = (bodyA.isStatic || !enableRotation) ? glm::vec3(0.0f)
                                      : glm::cross(bodyA.inverseInertiaTensorWorld * glm::cross(rA, tangent), rA);
    glm::vec3 angB_t = (bodyB.isStatic || !enableRotation) ? glm::vec3(0.0f)
                                      : glm::cross(bodyB.inverseInertiaTensorWorld * glm::cross(rB, tangent), rB);

    float invMassTangent = invMassA + invMassB + glm::dot(tangent, angA_t + angB_t);
    if (invMassTangent > 0.0f) {
      float jt = -glm::dot(relativeVelocity, tangent) / invMassTangent;

      float muStatic = std::sqrt(bodyA.staticFriction * bodyB.staticFriction);
      float muDynamic = std::sqrt(bodyA.dynamicFriction * bodyB.dynamicFriction);

      // Coulomb law: clamp friction by normal impulse
      float maxFriction = muStatic * jn;
      if (std::abs(jt) > maxFriction) {
        jt = (jt > 0.0f ? 1.0f : -1.0f) * (muDynamic * jn);
      }

      glm::vec3 frictionImpulse = jt * tangent;
      if (!bodyA.isStatic) {
        bodyA.velocity -= frictionImpulse * bodyA.inverseMass;
        if (enableRotation) bodyA.angularVelocity -= bodyA.inverseInertiaTensorWorld * glm::cross(rA, frictionImpulse);
      }
      if (!bodyB.isStatic) {
        bodyB.velocity += frictionImpulse * bodyB.inverseMass;
        if (enableRotation) bodyB.angularVelocity += bodyB.inverseInertiaTensorWorld * glm::cross(rB, frictionImpulse);
      }
    }
  }
}

void resolveContact(RigidBodyComponent& bodyA, RigidBodyComponent& bodyB, const ContactManifold& contact) {
  TransformComponent dummyA;
  TransformComponent dummyB;
  dummyA.position = contact.point;
  dummyB.position = contact.point;
  resolveContact(bodyA, dummyA, bodyB, dummyB, contact);
}

void integrateRigidBody(RigidBodyComponent& body, TransformComponent& transform, const glm::vec3& gravity,
                        float dt, bool enableRotation) {
  if (body.isStatic || body.inverseMass <= 0.0f) {
    return;
  }

  // Linear integration
  if (body.useGravity) {
    body.velocity += gravity * dt;
  }
  body.velocity *= std::max(0.0f, 1.0f - body.linearDamping);
  transform.position += body.velocity * dt;

  if (!enableRotation) return;

  // Angular integration
  body.angularVelocity *= std::max(0.0f, 1.0f - body.angularDamping);

  glm::vec3 w = body.angularVelocity;
  float wLen = glm::length(w);
  if (wLen > 1e-6f) {
    glm::quat wQuat(0.0f, w.x, w.y, w.z);
    transform.rotation = transform.rotation + (0.5f * dt) * (wQuat * transform.rotation);
    transform.rotation = glm::normalize(transform.rotation);
  }

  body.updateInertiaWorld(transform.rotation);
}

bool intersectRaySphere(const Ray& ray, const glm::vec3& center, float radius, float& tHit, glm::vec3& hitNormal) {
  glm::vec3 m = ray.origin - center;
  float b = glm::dot(m, ray.direction);
  float c = glm::dot(m, m) - radius * radius;

  if (c > 0.0f && b > 0.0f) return false;

  float discr = b * b - c;
  if (discr < 0.0f) return false;

  float t = -b - std::sqrt(discr);
  if (t < 0.0f) t = 0.0f;

  tHit = t;
  hitNormal = glm::normalize((ray.origin + ray.direction * t) - center);
  return true;
}

bool intersectRayOBB(const Ray& ray, const glm::vec3& center, const glm::quat& rotation,
                     const glm::vec3& halfExtents, float& tHit, glm::vec3& hitNormal) {
  glm::mat3 R = glm::mat3_cast(rotation);
  glm::mat3 invR = glm::transpose(R);

  // Transform ray to local box space
  glm::vec3 localOrigin = invR * (ray.origin - center);
  glm::vec3 localDir = invR * ray.direction;

  float tMin = 0.0f;
  float tMax = std::numeric_limits<float>::max();
  glm::vec3 localHitNormal(0.0f);

  for (int i = 0; i < 3; ++i) {
    if (std::abs(localDir[i]) < 1e-6f) {
      if (localOrigin[i] < -halfExtents[i] || localOrigin[i] > halfExtents[i]) {
        return false;
      }
    } else {
      float invD = 1.0f / localDir[i];
      float t1 = (-halfExtents[i] - localOrigin[i]) * invD;
      float t2 = (halfExtents[i] - localOrigin[i]) * invD;
      float sign = -1.0f;
      if (t1 > t2) {
        std::swap(t1, t2);
        sign = 1.0f;
      }

      if (t1 > tMin) {
        tMin = t1;
        localHitNormal = glm::vec3(0.0f);
        localHitNormal[i] = sign;
      }
      tMax = std::min(tMax, t2);

      if (tMin > tMax) return false;
    }
  }

  tHit = tMin;
  hitNormal = glm::normalize(R * localHitNormal);
  return true;
}

}  // namespace simforge
