#pragma once
#include "glm/glm.hpp"

float geod_dict(const glm::vec3 &m0,
                const glm::vec3 &m1,
                const glm::vec3 &v0,
                const glm::vec3 &v1);

float ang_dict(const glm::vec3 &m0,
               const glm::vec3 &m1,
               const glm::vec3 &v0,
               const glm::vec3 &v1);
