#include <iostream>

#include "decomp.h"
#include "mesh.h"

int main(int argc, char **argv) {
  std::string load_path = ASSETS_DIR "texturedknot.ply";
  std::string store_path = "*";
  if (argc > 1) {
    load_path = argv[1];
    if (argc == 3) {
      store_path = argv[2];
    } else if (argc > 3) {
      std::cout << "Usage: " << argv[0] << " <load_path> [<store_path>]"
                << std::endl;
      return 1;
    }
  } else {
    std::cout << "Usage: " << argv[0] << " <load_path> [<store_path>]"
              << std::endl;
    return 1;
  }
  // load_path ends with ".ply"
  if (load_path.size() < 4 ||
      load_path.substr(load_path.size() - 4) != ".ply") {
    std::cout << "load_path must end with .ply";
    return 1;
  }
  if (store_path == "*") {
    store_path = load_path.substr(0, load_path.size() - 4) + "_decomp.ply";
  }
  Mesh mesh;
  mesh.load_ply(load_path);
  auto result = K_decomp_hierarchy(mesh);
  store_combined_ply(store_path, result);
}
