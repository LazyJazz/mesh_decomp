#pragma once
#include "mesh.h"

std::vector<Mesh> K_decomp(const Mesh &mesh);

std::vector<Mesh> K_decomp_hierarchy(const Mesh &mesh);
