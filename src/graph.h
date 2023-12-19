#pragma once

#include "map"
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
