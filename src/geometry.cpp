#include "geometry.h"

namespace {
const float eps = 1e-6f;
}

float geod_dist(const glm::vec3 &m0,
                const glm::vec3 &m1,
                const glm::vec3 &v0,
                const glm::vec3 &v1) {
  auto c0 = (m0 + m1 + v0) / 3.f;
  auto c1 = (m0 + m1 + v1) / 3.f;
  auto d = m1 - m0;
  if (glm::length(d) < eps) {
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

float ang_dist(const glm::vec3 &m0,
               const glm::vec3 &m1,
               const glm::vec3 &v0,
               const glm::vec3 &v1) {
  return AngDist(0.5f)(m0, m1, v0, v1);
}

float dual_face_ang(const glm::vec3 &m0,
                    const glm::vec3 &m1,
                    const glm::vec3 &v0,
                    const glm::vec3 &v1) {
  auto c0 = (m0 + m1 + v0) / 3.f;
  auto c1 = (m0 + m1 + v1) / 3.f;
  auto d = m1 - m0;
  auto n0 = glm::cross(d, v0 - m1);
  auto n1 = glm::cross(-d, v1 - m0);
  if (glm::length(n0) < eps)
    return 0;
  if (glm::length(n1) < eps)
    return 0;
  n0 = glm::normalize(n0);
  n1 = glm::normalize(n1);
  float cos_ang = glm::dot(n0, n1);
  cos_ang = glm::max(-1.f, glm::min(1.f, cos_ang));
  return glm::acos(cos_ang);
}

float AngDist::operator()(const glm::vec3 &m0,
                          const glm::vec3 &m1,
                          const glm::vec3 &v0,
                          const glm::vec3 &v1) const {
  auto c0 = (m0 + m1 + v0) / 3.f;
  auto c1 = (m0 + m1 + v1) / 3.f;
  auto d = m1 - m0;
  auto n0 = glm::cross(d, v0 - m1);
  auto n1 = glm::cross(-d, v1 - m0);
  if (glm::length(n0) < eps)
    return 0;
  if (glm::length(n1) < eps)
    return 0;
  n0 = glm::normalize(n0);
  n1 = glm::normalize(n1);
  float res = 1.0f - glm::dot(n0, n1);
  if (glm::dot(c1 - c0, n0) < 0) {
    res = eta * res;
  }
  return res;
}
