#pragma once

#include "algorithm"
#include "map"
#include "queue"
#include "vector"

template <class Content = int>
struct Graph {
  struct Edge {
    int dst;
    Content content;
  };
  std::map<int, std::vector<Edge>> edges;

  void add_edge(int src, int dst, Content content) {
    edges[src].push_back({dst, content});
  }
};

std::map<int, float> Dijkstra(const Graph<float> &graph, int start);

std::map<int, float> Dijkstra(const Graph<float> &graph,
                              const std::vector<int> &starts);

std::map<int, std::map<int, float>> FloydWarshall(const Graph<float> &graph);

// Make a function that finds graph center
// Path: src/graph.cpp
int FindGraphCenter(const Graph<float> &graph);

struct NetworkFlowGraph {
  struct Edge {
    int dst;
    float capacity;
    float flow;
    int nxt;
  };
  std::map<int, int> first;
  std::vector<Edge> edges;

  NetworkFlowGraph() {
    edges.resize(2);
  }

  void add_edge(int u, int v, float c) {
    edges.push_back({v, c, 0, first[u]});
    first[u] = edges.size() - 1;
    edges.push_back({u, 0, 0, first[v]});
    first[v] = edges.size() - 1;
  }
};

float Dinic(NetworkFlowGraph &graph, int source, int tank);
