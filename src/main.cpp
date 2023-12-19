#include <iostream>

#include "decomp.h"
#include "mesh.h"

int main() {
  Mesh mesh;
  mesh.load_ply(ASSETS_DIR "cat.ply");
  K_decomp(mesh);
}
