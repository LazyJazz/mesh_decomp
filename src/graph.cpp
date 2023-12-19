#include "graph.h"

#include <functional>

#include "set"

std::map<int, float> Dijkstra(const Graph<float> &graph, int start) {
  return Dijkstra(graph, std::vector<int>{start});
}

std::map<int, float> Dijkstra(const Graph<float> &graph,
                              const std::vector<int> &starts) {
  std::map<int, float> result;
  const float inf = std::numeric_limits<float>::infinity();
  for (auto &x : graph.edges) {
    result[x.first] = inf;
  }
  struct State {
    int x;
    float d;
    bool operator<(const State &b) const {
      return d > b.d;
    }
  };
  std::priority_queue<State> q;
  for (auto start : starts) {
    result[start] = 0;
    q.push({start, 0});
  }
  while (!q.empty()) {
    auto [x, d] = q.top();
    q.pop();
    if (d > result[x])
      continue;
    for (auto &edge : graph.edges.at(x)) {
      int y = edge.dst;
      float w = edge.content;
      if (result[y] > result[x] + w) {
        result[y] = result[x] + w;
        q.push({y, result[y]});
      }
    }
  }
  return result;
}

std::map<int, std::map<int, float>> FloydWarshall(const Graph<float> &graph) {
  std::map<int, int> node_map;
  std::vector<int> node_list;

  auto get_node_index = [&](int node) {
    if (!node_map.count(node)) {
      int id = node_list.size();
      node_map[node] = id;
      node_list.push_back(node);
      return id;
    }
    return node_map[node];
  };

  for (auto &node : graph.edges) {
    get_node_index(node.first);
    for (auto &edge : node.second) {
      get_node_index(edge.dst);
    }
  }

  std::vector<std::vector<float>> dist(
      node_list.size(),
      std::vector<float>(node_list.size(),
                         std::numeric_limits<float>::infinity()));

  for (auto &node : graph.edges) {
    int x = get_node_index(node.first);
    dist[x][x] = 0;
    for (auto &edge : node.second) {
      int y = get_node_index(edge.dst);
      dist[x][y] = edge.content;
    }
  }

  for (int k = 0; k < node_list.size(); k++) {
    for (int i = 0; i < node_list.size(); i++) {
      for (int j = 0; j < node_list.size(); j++) {
        dist[i][j] = std::min(dist[i][j], dist[i][k] + dist[k][j]);
      }
    }
    printf("Floyd: %d/%d\n", k, node_list.size());
  }

  std::map<int, std::map<int, float>> result;
  for (int i = 0; i < node_list.size(); i++) {
    for (int j = 0; j < node_list.size(); j++) {
      result[node_list[i]][node_list[j]] = dist[i][j];
    }
  }
  return result;
}

int FindGraphCenter(const Graph<float> &graph) {
  auto dist = DijkstraComplete(graph);
  int result = -1;
  float min_dist = std::numeric_limits<float>::infinity();
  for (auto &node : dist) {
    float sum_dist = 0;
    for (auto &x : node.second) {
      sum_dist += x.second;
    }
    if (sum_dist < min_dist) {
      min_dist = sum_dist;
      result = node.first;
    }
  }
  return result;
}

int FindGraphCenterLocal(const Graph<float> &graph) {
  int x = graph.edges.begin()->first;
  std::map<int, float> value;
  auto get_value = [&](int x) {
    if (!value.count(x)) {
      auto dist = Dijkstra(graph, x);
      float sum_dist = 0;
      for (auto &node : dist) {
        sum_dist += node.second;
      }
      value[x] = sum_dist;
    }
    return value[x];
  };

  bool changed = true;
  while (changed) {
    changed = false;
    float value_x = get_value(x);
    for (auto &edge : graph.edges.at(x)) {
      float value_y = get_value(edge.dst);
      if (value_y < value_x) {
        x = edge.dst;
        changed = true;
        break;
      }
    }
  }

  return x;
}

float Dinic(NetworkFlowGraph &graph, int source, int tank) {
  const auto inf = std::numeric_limits<float>::infinity();
  const auto eps = 1e-6f;
  std::map<int, int> dist;
  for (auto &node : graph.first) {
    dist[node.first] = -1;
  }

  auto BFS = [&]() -> bool {
    std::queue<int> q;
    q.push(tank);
    dist[tank] = 0;
    while (!q.empty()) {
      auto x = q.front();
      q.pop();
      if (x == source) {
        return true;
      }
      for (int i = graph.first[x]; i; i = graph.edges[i].nxt) {
        auto &edge = graph.edges[i];
        auto &counter_edge = graph.edges[i ^ 1];
        if (counter_edge.capacity > counter_edge.flow + eps &&
            dist[edge.dst] == -1) {
          dist[edge.dst] = dist[x] + 1;
          q.push(edge.dst);
        }
      }
    }
    return false;
  };

  std::function<float(int, float)> DFS;
  DFS = [&](int x, float max_flow) {
    if (x == tank) {
      return max_flow;
    }
    float res = 0;
    for (int i = graph.first[x]; i; i = graph.edges[i].nxt) {
      auto &edge = graph.edges[i];
      if (edge.capacity - edge.flow > eps && dist[edge.dst] == dist[x] - 1) {
        float flow =
            DFS(edge.dst, std::min(max_flow, edge.capacity - edge.flow));
        edge.flow += flow;
        graph.edges[i ^ 1].flow -= flow;
        res += flow;
        max_flow -= flow;
        if (max_flow < eps)
          break;
      }
    }
    return res;
  };

  float res = 0.0f;

  while (BFS()) {
    res += DFS(source, inf);
    for (auto &node : graph.first) {
      dist[node.first] = -1;
    }
    printf("%f\n", res);
  }

  return res;
}

std::map<int, std::map<int, float>> DijkstraComplete(
    const Graph<float> &graph) {
  std::map<int, std::map<int, float>> result;
  int total_size = graph.edges.size();
  int completed = 0;
  for (auto &node : graph.edges) {
    result[node.first] = Dijkstra(graph, node.first);
    completed++;
    printf("DijkstraComplete: %d/%d\n", completed, total_size);
  }
  return result;
}
