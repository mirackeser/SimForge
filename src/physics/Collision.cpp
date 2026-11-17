#include "physics/Collision.hpp"

#include <algorithm>
#include <cmath>
#include <glm/gtc/quaternion.hpp>
#include <limits>
#include <stdexcept>
#include <vector>

#include "core/Validation.hpp"
#include "physics/ContactSolver.hpp"

namespace simforge {

namespace {

// Clamps point to box half-extents in local coordinate space
glm::vec3 closestPointOnLocalBox(const glm::vec3& localPoint, const glm::vec3& halfExtents) {
  return glm::clamp(localPoint, -halfExtents, halfExtents);
}

std::vector<glm::vec3> clipPolygon(const std::vector<glm::vec3>& input, const glm::vec3& normal,
                                   float offset) {
  std::vector<glm::vec3> output;
  if (input.empty()) return output;
  auto previous = input.back();
  float previousDistance = glm::dot(normal, previous) - offset;
  for (const auto& current : input) {
    const float distance = glm::dot(normal, current) - offset;
    if ((distance <= 0.0f) != (previousDistance <= 0.0f)) {
      const float fraction = previousDistance / (previousDistance - distance);
      output.push_back(previous + fraction * (current - previous));
    }
    if (distance <= 0.0f) output.push_back(current);
    previous = current;
    previousDistance = distance;
  }
  return output;
}

glm::vec3 support(const glm::vec3& center, const glm::mat3& axes, const glm::vec3& half,
                  const glm::vec3& direction) {
  glm::vec3 result = center;
  for (int i = 0; i < 3; ++i) {
    const float projection = glm::dot(axes[i], direction);
    if (std::abs(projection) > 1e-6f) result += axes[i] * std::copysign(half[i], projection);
  }
  return result;
}

glm::vec3 edgeContact(const TransformComponent& a, const ColliderComponent& ca, int edgeA,
                      const TransformComponent& b, const ColliderComponent& cb, int edgeB,
                      const glm::vec3& normal) {
  const glm::mat3 ra = glm::mat3_cast(a.rotation), rb = glm::mat3_cast(b.rotation);
  glm::vec3 centerA = a.position, centerB = b.position;
  for (int i = 0; i < 3; ++i) {
    if (i != edgeA) centerA += ra[i] * std::copysign(ca.halfExtents[i], glm::dot(ra[i], normal));
    if (i != edgeB) centerB += rb[i] * std::copysign(cb.halfExtents[i], glm::dot(rb[i], -normal));
  }
  const glm::vec3 startA = centerA - ra[edgeA] * ca.halfExtents[edgeA];
  const glm::vec3 startB = centerB - rb[edgeB] * cb.halfExtents[edgeB];
  const glm::vec3 da = 2.0f * ra[edgeA] * ca.halfExtents[edgeA];
  const glm::vec3 db = 2.0f * rb[edgeB] * cb.halfExtents[edgeB];
  const glm::vec3 offset = startA - startB;
  const float aa = glm::dot(da, da), bb = glm::dot(db, db), ab = glm::dot(da, db);
  const float ao = glm::dot(da, offset), bo = glm::dot(db, offset);
  const float denominator = aa * bb - ab * ab;
  float u = denominator > 1e-8f ? glm::clamp((ab * bo - ao * bb) / denominator, 0.0f, 1.0f) : 0.0f;
  float v = (ab * u + bo) / bb;
  if (v < 0.0f) {
    v = 0.0f;
    u = glm::clamp(-ao / aa, 0.0f, 1.0f);
  } else if (v > 1.0f) {
    v = 1.0f;
    u = glm::clamp((ab - ao) / aa, 0.0f, 1.0f);
  }
  return 0.5f * (startA + u * da + startB + v * db);
}

void addFaceContacts(ContactManifold& contact, const TransformComponent& reference,
                     const ColliderComponent& referenceShape, const TransformComponent& incident,
                     const ColliderComponent& incidentShape, const glm::vec3& normal) {
  const glm::mat3 referenceAxes = glm::mat3_cast(reference.rotation);
  const glm::mat3 incidentAxes = glm::mat3_cast(incident.rotation);
  int faceAxis = 0, incidentAxis = 0;
  for (int i = 1; i < 3; ++i) {
    if (std::abs(glm::dot(referenceAxes[i], normal)) >
        std::abs(glm::dot(referenceAxes[faceAxis], normal)))
      faceAxis = i;
    if (std::abs(glm::dot(incidentAxes[i], normal)) >
        std::abs(glm::dot(incidentAxes[incidentAxis], normal)))
      incidentAxis = i;
  }
  const glm::vec3 faceCenter = reference.position + normal * referenceShape.halfExtents[faceAxis];
  const float sign = glm::dot(incidentAxes[incidentAxis], normal) > 0.0f ? -1.0f : 1.0f;
  const glm::vec3 incidentCenter = incident.position + sign * incidentAxes[incidentAxis] *
                                                           incidentShape.halfExtents[incidentAxis];
  const int u = (incidentAxis + 1) % 3, v = (incidentAxis + 2) % 3;
  const glm::vec3 du = incidentAxes[u] * incidentShape.halfExtents[u];
  const glm::vec3 dv = incidentAxes[v] * incidentShape.halfExtents[v];
  std::vector<glm::vec3> polygon = {incidentCenter - du - dv, incidentCenter + du - dv,
                                    incidentCenter + du + dv, incidentCenter - du + dv};
  for (int i = 0; i < 3; ++i) {
    if (i == faceAxis) continue;
    for (float side : {-1.0f, 1.0f}) {
      const glm::vec3 plane = side * referenceAxes[i];
      polygon = clipPolygon(polygon, plane,
                            glm::dot(plane, reference.position) + referenceShape.halfExtents[i]);
    }
  }
  glm::vec3 sum(0.0f);
  for (const auto& point : polygon) {
    const float separation = glm::dot(normal, point - faceCenter);
    if (separation > 1e-4f || contact.pointCount == contact.points.size()) continue;
    const glm::vec3 midpoint = point - 0.5f * separation * normal;
    bool duplicate = false;
    for (std::size_t i = 0; i < contact.pointCount; ++i) {
      const auto delta = midpoint - contact.points[i];
      if (glm::dot(delta, delta) < 1e-8f) duplicate = true;
    }
    if (!duplicate) {
      contact.points[contact.pointCount++] = midpoint;
      sum += midpoint;
    }
  }
  if (contact.pointCount != 0) contact.point = sum / static_cast<float>(contact.pointCount);
}

}  // namespace

std::optional<ContactManifold> collideSphereSphere(const TransformComponent& a,
                                                   const ColliderComponent& ca,
                                                   const TransformComponent& b,
                                                   const ColliderComponent& cb) {
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

std::optional<ContactManifold> collideSphereBox(const TransformComponent& sphere,
                                                const ColliderComponent& cs,
                                                const TransformComponent& box,
                                                const ColliderComponent& cb) {
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
    m.point = box.position +
              boxRot * (localSpherePos + localNormal * glm::dot(pen, glm::abs(localNormal)));
  }

  return m;
}

std::optional<ContactManifold> collideBoxBox(const TransformComponent& a,
                                             const ColliderComponent& ca,
                                             const TransformComponent& b,
                                             const ColliderComponent& cb) {
  glm::mat3 RA = glm::mat3_cast(a.rotation);
  glm::mat3 RB = glm::mat3_cast(b.rotation);

  glm::vec3 D = b.position - a.position;

  // 15 candidate separating axes:
  // 3 axes of A, 3 axes of B, 9 cross products
  glm::vec3 axes[15];
  int edgeA[15]{}, edgeB[15]{};
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
        edgeA[axisCount] = i;
        edgeB[axisCount] = j;
        axes[axisCount++] = crossAxis / std::sqrt(lenSq);
      }
    }
  }

  float minOverlap = std::numeric_limits<float>::max();
  glm::vec3 bestAxis(0.0f, 1.0f, 0.0f);
  int bestIndex = 0;

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
      bestIndex = i;
    }
  }

  // Ensure normal points from A to B
  if (glm::dot(bestAxis, D) < 0.0f) {
    bestAxis = -bestAxis;
  }

  ContactManifold m;
  m.normal = bestAxis;
  m.penetration = minOverlap;

  if (bestIndex < 3)
    addFaceContacts(m, a, ca, b, cb, bestAxis);
  else if (bestIndex < 6)
    addFaceContacts(m, b, cb, a, ca, -bestAxis);
  if (bestIndex >= 6)
    m.point = edgeContact(a, ca, edgeA[bestIndex], b, cb, edgeB[bestIndex], bestAxis);
  else if (m.pointCount == 0) {
    m.point = 0.5f * (support(a.position, RA, ca.halfExtents, bestAxis) +
                      support(b.position, RB, cb.halfExtents, -bestAxis));
  }

  return m;
}

void resolveContact(RigidBodyComponent& bodyA, TransformComponent& transformA,
                    RigidBodyComponent& bodyB, TransformComponent& transformB,
                    const ContactManifold& contact, bool enableFriction, bool enableRotation) {
  ContactSolverState state;
  solveContactIteration(bodyA, transformA, bodyB, transformB, contact, state, enableFriction,
                        enableRotation);
}

void integrateRigidBody(RigidBodyComponent& body, TransformComponent& transform,
                        const glm::vec3& gravity, float dt, bool enableRotation) {
  if (!std::isfinite(dt) || dt < 0.0f) throw std::invalid_argument("Invalid integration timestep");
  if (body.isStatic || body.inverseMass <= 0.0f) {
    return;
  }

  // Linear integration
  if (body.useGravity) {
    body.velocity += gravity * dt;
  }
  body.velocity *= std::exp(-std::max(0.0f, body.linearDamping) * dt);
  transform.position += body.velocity * dt;

  if (!enableRotation) return;

  // Angular integration
  body.angularVelocity *= std::exp(-std::max(0.0f, body.angularDamping) * dt);

  glm::vec3 w = body.angularVelocity;
  float wLen = glm::length(w);
  if (wLen > 1e-6f) {
    transform.rotation = glm::angleAxis(wLen * dt, w / wLen) * transform.rotation;
    transform.rotation = glm::normalize(transform.rotation);
  }

  body.updateInertiaWorld(transform.rotation);
}

bool intersectRaySphere(const Ray& ray, const glm::vec3& center, float radius, float& tHit,
                        glm::vec3& hitNormal) {
  const float lengthSquared = glm::dot(ray.direction, ray.direction);
  if (!finite(ray.origin) || !finite(ray.direction) || !finite(center) || !std::isfinite(radius) ||
      radius <= 0.0f || std::abs(lengthSquared - 1.0f) > 1e-3f)
    return false;
  const glm::vec3 offset = ray.origin - center;
  const float projection = glm::dot(offset, ray.direction);
  const float discriminant = projection * projection - glm::dot(offset, offset) + radius * radius;
  if (discriminant < 0.0f) return false;
  const float root = std::sqrt(discriminant);
  float distance = -projection - root;
  if (distance < 0.0f) distance = -projection + root;
  if (distance < 0.0f) return false;
  tHit = distance;
  hitNormal = (ray.origin + ray.direction * distance - center) / radius;
  return true;
}

bool intersectRayOBB(const Ray& ray, const glm::vec3& center, const glm::quat& rotation,
                     const glm::vec3& halfExtents, float& tHit, glm::vec3& hitNormal) {
  if (!finite(ray.origin) || !finite(ray.direction) || !finite(center) || !finite(rotation) ||
      !finite(halfExtents) || glm::any(glm::lessThanEqual(halfExtents, glm::vec3(0.0f))) ||
      std::abs(glm::dot(rotation, rotation) - 1.0f) > 1e-3f ||
      std::abs(glm::dot(ray.direction, ray.direction) - 1.0f) > 1e-3f)
    return false;
  const glm::mat3 axes = glm::mat3_cast(rotation);
  const glm::vec3 origin = glm::transpose(axes) * (ray.origin - center);
  const glm::vec3 direction = glm::transpose(axes) * ray.direction;
  float entry = -std::numeric_limits<float>::infinity();
  float exit = std::numeric_limits<float>::infinity();
  glm::vec3 entryNormal(0.0f), exitNormal(0.0f);
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(direction[axis]) < 1e-7f) {
      if (std::abs(origin[axis]) > halfExtents[axis]) return false;
      continue;
    }
    float near = (-halfExtents[axis] - origin[axis]) / direction[axis];
    float far = (halfExtents[axis] - origin[axis]) / direction[axis];
    float sign = -1.0f;
    if (near > far) {
      std::swap(near, far);
      sign = 1.0f;
    }
    if (near > entry) {
      entry = near;
      entryNormal = glm::vec3(0.0f);
      entryNormal[axis] = sign;
    }
    if (far < exit) {
      exit = far;
      exitNormal = glm::vec3(0.0f);
      exitNormal[axis] = -sign;
    }
    if (entry > exit) return false;
  }
  if (exit < 0.0f) return false;
  tHit = entry >= 0.0f ? entry : exit;
  hitNormal = axes * (entry >= 0.0f ? entryNormal : exitNormal);
  return true;
}

}  // namespace simforge
