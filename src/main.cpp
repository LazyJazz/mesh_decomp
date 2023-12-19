#include <iostream>

#include "decomp.h"
#include "mesh.h"

int main() {
  std::string load_path = ASSETS_DIR "cat.ply";
  Mesh mesh;
  mesh.load_ply(load_path);
  auto result = K_decomp(mesh);
  store_combined_ply(load_path.substr(0, load_path.size() - 4) + "_decomp.ply",
                     result);
}
