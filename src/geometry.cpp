#include "geometry.h"

float geod_dict(const glm::vec3 &m0,
                const glm::vec3 &m1,
                const glm::vec3 &v0,
                const glm::vec3 &v1) {
  auto c0 = (m0 + m1 + v0) / 3.f;
  auto c1 = (m0 + m1 + v1) / 3.f;
  auto d = m1 - m0;
  if (glm::length(d) < 1e-6f) {
    return glm::length(c0 - m0) + glm::length(c1 - m1);
  }
  auto n = glm::normalize(d);
  auto p0 = glm::dot(c0 - m0, n) * n + m0;
  auto p1 = glm::dot(c1 - m0, n) * n + m0;
  auto r0 = glm::length(c0 - p0);
  auto r1 = glm::length(c1 - p1);
  auto c = glm::length(p1 - p0);
  return sqrt((r0 + r1) * (r0 + r1) + c * c);
}

float ang_dict(const glm::vec3 &m0,
               const glm::vec3 &m1,
               const glm::vec3 &v0,
               const glm::vec3 &v1) {
  auto c0 = (m0 + m1 + v0) / 3.f;
  auto c1 = (m0 + m1 + v1) / 3.f;
  auto d = m1 - m0;
  auto n0 = glm::cross(d, v0 - m1);
  auto n1 = glm::cross(-d, v1 - m0);
  return 0;
}
