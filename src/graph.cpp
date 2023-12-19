#include "graph.h"

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
  auto dist = FloydWarshall(graph);
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
