#include "decomp.h"

#include "set"

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
  const float inf = 1e12f;
  auto origin_graph = mesh.build_dual_graph(0.3);
  auto graph = Mesh::convert_graph(origin_graph, 0.7);
  auto reps = GetReps(graph);

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
  assemblers.resize(reps.size() + 1);
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
    int min_rep = -1;
    int second_min_rep = -1;
    for (int i = 0; i < reps.size(); i++) {
      float dist = shortest_paths[i][x];
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

    if (second_min_dist < 1.2f * min_dist) {
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
              subgraph.add_edge(face_id, edge.dst, edge.content.ang_dist);
            }
          }
        }
      }

      if (num_edges) {
        avg_ang_dist /= (float)num_edges;
        for (auto &node : subgraph.first) {
          if (node.first == source || node.first == tank) {
            continue;
          }
          for (int x = node.second; x; x = subgraph.edges[x].nxt) {
            auto &edge = subgraph.edges[x];
            if (edge.dst == source || edge.dst == tank) {
              continue;
            }
            if (edge.capacity != 0.0f) {
              edge.capacity = 1.0f / (1.0f + edge.capacity / avg_ang_dist);
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
  results.reserve(reps.size() + 1);
  for (auto &assembler : assemblers) {
    results.emplace_back(assembler.GetMesh());
  }

  return results;
}
