#include <iostream>

#include "mesh.h"

int main() {
  Mesh mesh;
  mesh.load_ply(ASSETS_DIR "bunny.ply");
  mesh.store_ply(ASSETS_DIR "bunny2.ply", glm::vec3{0, 1, 0});
}
