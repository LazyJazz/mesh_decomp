#pragma once
#include "glm/glm.hpp"

float geod_dist(const glm::vec3 &m0,
                const glm::vec3 &m1,
                const glm::vec3 &v0,
                const glm::vec3 &v1);

float ang_dist(const glm::vec3 &m0,
               const glm::vec3 &m1,
               const glm::vec3 &v0,
               const glm::vec3 &v1);

struct AngDist {
  float eta;
  explicit AngDist(float eta = 0.5f) : eta(eta) {
  }
  float operator()(const glm::vec3 &m0,
                   const glm::vec3 &m1,
                   const glm::vec3 &v0,
                   const glm::vec3 &v1) const;
};

float dual_face_ang(const glm::vec3 &m0,
                    const glm::vec3 &m1,
                    const glm::vec3 &v0,
                    const glm::vec3 &v1);
