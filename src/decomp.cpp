#include "decomp.h"

#include "set"

namespace {
std::vector<int> GetReps(const Graph<float> &graph) {
  std::vector<int> reps;
  reps.push_back(FindGraphCenterLocal(graph));

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
  const float inf = 1e12f;
  auto origin_graph = mesh.build_dual_graph(0.5);
  auto graph = Mesh::convert_graph(origin_graph, 0.5);
  auto reps = GetReps(graph);
  float geod_dist_mean, ang_dist_mean;
  Mesh::graph_dist_mean(origin_graph, geod_dist_mean, ang_dist_mean);

  for (auto rep : reps) {
    printf("%d\n", rep);
  }

  std::vector<std::map<int, float>> shortest_paths;
  shortest_paths.reserve(reps.size());

  for (auto rep : reps) {
    shortest_paths.push_back(Dijkstra(graph, rep));
  }

  std::vector<std::pair<int, int>> belongs(mesh.faces().size(), {-1, -1});
  std::vector<MeshAssembler> assemblers;
  assemblers.resize(reps.size() + 2);
  auto &vertices = mesh.vertices();
  auto &faces = mesh.faces();

  auto assign_face = [&](int rep_id, int face_id) {
    assemblers[rep_id].AddFace(vertices[faces[face_id].v1],
                               vertices[faces[face_id].v2],
                               vertices[faces[face_id].v3]);
  };

  for (auto &node : graph.edges) {
    int x = node.first;
    // Find the first two shortest
    float min_dist = std::numeric_limits<float>::infinity();
    float second_min_dist = std::numeric_limits<float>::infinity();
    float sum_dist_inv = 0.0f;
    int min_rep = -1;
    int second_min_rep = -1;
    for (int i = 0; i < reps.size(); i++) {
      float dist = shortest_paths[i][x];
      sum_dist_inv += 1.0f / dist;
      if (dist < min_dist) {
        second_min_dist = min_dist;
        second_min_rep = min_rep;
        min_dist = dist;
        min_rep = i;
      } else if (dist < second_min_dist) {
        second_min_dist = dist;
        second_min_rep = i;
      }
    }

    if (min_dist == 0.0f) {
      belongs[x] = {min_rep, min_rep};
      assign_face(min_rep, x);
      continue;
    }

    float min_prob = (1.0f / min_dist) / sum_dist_inv;
    float second_min_prob = (1.0f / second_min_dist) / sum_dist_inv;

    if (min_prob - second_min_prob < 0.1) {
      if (min_rep > second_min_rep) {
        std::swap(min_rep, second_min_rep);
      }
      belongs[x] = {min_rep, second_min_rep};
    } else {
      belongs[x] = {min_rep, min_rep};
      assign_face(min_rep, x);
    }
  }

  int source = faces.size();
  int tank = faces.size() + 1;

  for (int i = 0; i < reps.size(); i++) {
    for (int j = i + 1; j < reps.size(); j++) {
      std::set<int> face_set;
      auto avg_ang_dist = 0.0f;
      int num_edges = 0;
      NetworkFlowGraph subgraph;
      for (int face_id = 0; face_id < faces.size(); face_id++) {
        if (belongs[face_id].first == i && belongs[face_id].second == j) {
          face_set.insert(face_id);
          for (auto &edge : origin_graph.edges[face_id]) {
            if (belongs[edge.dst].first == i && belongs[edge.dst].second == i) {
              subgraph.add_edge(source, face_id, inf);
            } else if (belongs[edge.dst].first == j &&
                       belongs[edge.dst].second == j) {
              subgraph.add_edge(face_id, tank, inf);
            } else if (belongs[edge.dst].first == i &&
                       belongs[edge.dst].second == j) {
              avg_ang_dist += edge.content.ang_dist;
              num_edges++;
              subgraph.add_edge(
                  face_id, edge.dst,
                  1.0f / (1.0f + edge.content.ang_dist / ang_dist_mean));
            }
          }
        }
      }

      Dinic(subgraph, source, tank);

      std::queue<int> q;
      q.push(source);
      while (!q.empty()) {
        int x = q.front();
        q.pop();
        for (int y = subgraph.first[x]; y; y = subgraph.edges[y].nxt) {
          auto &edge = subgraph.edges[y];
          if (edge.capacity - edge.flow > 1e-6f &&
              face_set.find(edge.dst) != face_set.end()) {
            q.push(edge.dst);
            assign_face(i, edge.dst);
            face_set.erase(edge.dst);
          }
        }
      }

      for (auto face : face_set) {
        assign_face(j, face);
      }
    }
  }

  std::vector<Mesh> temporal_results;
  temporal_results.reserve(reps.size() + 2);
  for (auto &assembler : assemblers) {
    temporal_results.emplace_back(assembler.GetMesh());
  }

  return temporal_results;
}

namespace {
float first_level_max_rep_dist;
float first_level_ang;
}  // namespace

std::vector<Mesh> K_decomp_hierarchy_kernel(const Mesh &mesh,
                                            float geod_dist_mean,
                                            float ang_dist_mean,
                                            int depth) {
  if (mesh.faces().empty() || mesh.vertices().empty()) {
    return {};
  }
  const float inf = 1e12f;
  auto ang_graph = mesh.build_ang_graph();
  if (ang_graph.edges.empty()) {
    return {mesh};
  }
  auto origin_graph = mesh.build_dual_graph(0.5f);
  if (!geod_dist_mean || !ang_dist_mean) {
    Mesh::graph_dist_mean(origin_graph, geod_dist_mean, ang_dist_mean);
  }
  auto graph =
      Mesh::convert_graph(origin_graph, 0.5f, geod_dist_mean, ang_dist_mean);
  auto reps = GetReps(graph);

  float max_ang = 0.0f;
  float min_ang = 1e9f;
  float max_rep_dist = 0.0f;
  float mean_dist = 0.0f;
  int num_edges = 0;

  for (auto rep : reps) {
    auto dijk_res = Dijkstra(graph, rep);
    for (auto rep2 : reps) {
      if (rep == rep2)
        continue;
      max_rep_dist = std::max(max_rep_dist, dijk_res[rep2]);
    }
  }

  for (auto &node : ang_graph.edges) {
    for (auto &edge : node.second) {
      max_ang = std::max(max_ang, edge.content);
      min_ang = std::min(min_ang, edge.content);
    }
  }

  for (auto &node : graph.edges) {
    for (auto &edge : node.second) {
      mean_dist += edge.content;
      num_edges++;
    }
  }

  if (!depth) {
    first_level_max_rep_dist = max_rep_dist;
    first_level_ang = max_ang - min_ang;
  }

  bool terminate = false;
  if (max_rep_dist / first_level_max_rep_dist < 0.4f) {
    terminate = true;
  }
  if ((max_ang - min_ang) < 1.5f) {
    terminate = true;
  }
  if (mean_dist < 0.3f) {
    terminate = true;
  }

  printf("max_rep_dist: %f\n", max_rep_dist);
  printf("max_ang: %f\n", max_ang);
  printf("min_ang: %f\n", min_ang);
  printf("mean_dist: %f\n", mean_dist / num_edges);
  // max_rep_dist / first_level_max_rep_dist
  printf("max_rep_dist / first_level_max_rep_dist: %f\n",
         max_rep_dist / first_level_max_rep_dist);
  // (max_ang - min_ang) / first_level_ang
  printf("(max_ang - min_ang) / first_level_ang: %f\n",
         (max_ang - min_ang) / first_level_ang);
  printf("terminate: %s\n", terminate ? "true" : "false");
  printf("===========");

  if (terminate && depth) {
    return {mesh};
  }

  for (auto rep : reps) {
    printf("%d\n", rep);
  }

  std::vector<std::map<int, float>> shortest_paths;
  shortest_paths.reserve(reps.size());

  for (auto rep : reps) {
    shortest_paths.push_back(Dijkstra(graph, rep));
  }

  std::vector<std::pair<int, int>> belongs(mesh.faces().size(), {-1, -1});
  std::vector<MeshAssembler> assemblers;
  assemblers.resize(reps.size() + 2);
  auto &vertices = mesh.vertices();
  auto &faces = mesh.faces();

  auto assign_face = [&](int rep_id, int face_id) {
    assemblers[rep_id].AddFace(vertices[faces[face_id].v1],
                               vertices[faces[face_id].v2],
                               vertices[faces[face_id].v3]);
  };

  for (auto &node : graph.edges) {
    int x = node.first;
    // Find the first two shortest
    float min_dist = std::numeric_limits<float>::infinity();
    float second_min_dist = std::numeric_limits<float>::infinity();
    float sum_dist_inv = 0.0f;
    int min_rep = -1;
    int second_min_rep = -1;
    for (int i = 0; i < reps.size(); i++) {
      float dist = shortest_paths[i][x];
      sum_dist_inv += 1.0f / dist;
      if (dist < min_dist) {
        second_min_dist = min_dist;
        second_min_rep = min_rep;
        min_dist = dist;
        min_rep = i;
      } else if (dist < second_min_dist) {
        second_min_dist = dist;
        second_min_rep = i;
      }
    }

    if (min_dist == 0.0f) {
      belongs[x] = {min_rep, min_rep};
      assign_face(min_rep, x);
      continue;
    }

    float min_prob = (1.0f / min_dist) / sum_dist_inv;
    float second_min_prob = (1.0f / second_min_dist) / sum_dist_inv;

    if (min_prob - second_min_prob < 0.1) {
      if (min_rep > second_min_rep) {
        std::swap(min_rep, second_min_rep);
      }
      belongs[x] = {min_rep, second_min_rep};
    } else {
      belongs[x] = {min_rep, min_rep};
      assign_face(min_rep, x);
    }
  }

  int source = faces.size();
  int tank = faces.size() + 1;

  for (int i = 0; i < reps.size(); i++) {
    for (int j = i + 1; j < reps.size(); j++) {
      std::set<int> face_set;
      auto avg_ang_dist = 0.0f;
      int num_edges = 0;
      NetworkFlowGraph subgraph;
      for (int face_id = 0; face_id < faces.size(); face_id++) {
        if (belongs[face_id].first == i && belongs[face_id].second == j) {
          face_set.insert(face_id);
          for (auto &edge : origin_graph.edges[face_id]) {
            if (belongs[edge.dst].first == i && belongs[edge.dst].second == i) {
              subgraph.add_edge(source, face_id, inf);
            } else if (belongs[edge.dst].first == j &&
                       belongs[edge.dst].second == j) {
              subgraph.add_edge(face_id, tank, inf);
            } else if (belongs[edge.dst].first == i &&
                       belongs[edge.dst].second == j) {
              avg_ang_dist += edge.content.ang_dist;
              num_edges++;
              subgraph.add_edge(
                  face_id, edge.dst,
                  1.0f / (1.0f + edge.content.ang_dist / ang_dist_mean));
            }
          }
        }
      }

      Dinic(subgraph, source, tank);

      std::queue<int> q;
      q.push(source);
      while (!q.empty()) {
        int x = q.front();
        q.pop();
        for (int y = subgraph.first[x]; y; y = subgraph.edges[y].nxt) {
          auto &edge = subgraph.edges[y];
          if (edge.capacity - edge.flow > 1e-6f &&
              face_set.find(edge.dst) != face_set.end()) {
            q.push(edge.dst);
            assign_face(i, edge.dst);
            face_set.erase(edge.dst);
          }
        }
      }

      for (auto face : face_set) {
        assign_face(j, face);
      }
    }
  }

  std::vector<Mesh> results;
  results.reserve(reps.size() + 2);
  for (auto &assembler : assemblers) {
    auto sub_results = K_decomp_hierarchy_kernel(
        assembler.GetMesh(), geod_dist_mean, ang_dist_mean, depth + 1);
    if (!sub_results.empty()) {
      for (auto &sub_result : sub_results) {
        results.push_back(sub_result);
      }
    }
  }

  return results;
}

std::vector<Mesh> K_decomp_hierarchy(const Mesh &mesh) {
  return K_decomp_hierarchy_kernel(mesh, 0.0f, 0.0f, 0);
}
