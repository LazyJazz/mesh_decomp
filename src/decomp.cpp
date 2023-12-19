#include "decomp.h"

namespace {
std::vector<int> GetReps(const Graph<float> &graph) {
  std::vector<int> reps;
  reps.push_back(FindGraphCenter(graph));

  if (reps[0] == -1) {
    std::cerr << "Graph is not connected" << std::endl;
    reps[0] = graph.edges.begin()->first;
  }

  float max_diff = -std::numeric_limits<float>::infinity();
  float last_G = 0.0f;

  float diff = 0.0f;
  do {
    auto dist = Dijkstra(graph, reps);
    int max_node = -1;
    float max_dist = -1;
    for (auto &node : graph.edges) {
      if (dist[node.first] > max_dist) {
        max_dist = dist[node.first];
        max_node = node.first;
      }
    }
    reps.push_back(max_node);
    diff = last_G - max_dist;
    max_diff = std::max(max_diff, diff);
    last_G = max_dist;
    printf("G: %lf\n", max_dist);
  } while (diff == max_diff);
  reps.resize(reps.size() - 2);
  return reps;
}
}  // namespace

std::vector<Mesh> K_decomp(const Mesh &mesh) {
  auto graph = mesh.build_dual_graph(0.5, 0.5);
  auto reps = GetReps(graph);
  for (auto rep : reps) {
    printf("%d\n", rep);
  }
  std::vector<Mesh> results;
  return results;
}
