#pragma once
#include "geometry.h"
#include "glm/glm.hpp"
#include "iostream"
#include "vector"

struct vec3_lt {
  bool operator()(const glm::vec3 &a, const glm::vec3 &b) const {
    if (a.x < b.x)
      return true;
    if (a.x > b.x)
      return false;
    if (a.y < b.y)
      return true;
    if (a.y > b.y)
      return false;
    return a.z < b.z;
  }
};

class Mesh {
 public:
  struct Face {
    int v1, v2, v3;
    Face(int v1 = 0, int v2 = 0, int v3 = 0) : v1(v1), v2(v2), v3(v3) {
    }
    [[nodiscard]] Face order() const {
      Face res{*this};
      if (res.v1 > res.v2)
        std::swap(res.v1, res.v2);
      if (res.v2 > res.v3)
        std::swap(res.v2, res.v3);
      if (res.v1 > res.v2)
        std::swap(res.v1, res.v2);
      return res;
    }

    bool operator<(const Face &other) const {
      Face a = order();
      Face b = other.order();
      if (a.v1 < b.v1)
        return true;
      if (a.v1 > b.v1)
        return false;
      if (a.v2 < b.v2)
        return true;
      if (a.v2 > b.v2)
        return false;
      return a.v3 < b.v3;
    }
  };

  struct Edge {
    int v1, v2;

    [[nodiscard]] Edge order() const {
      Edge res{*this};
      if (res.v1 > res.v2)
        std::swap(res.v1, res.v2);
      return res;
    }

    bool operator<(const Edge &other) const {
      Edge a = order();
      Edge b = other.order();
      if (a.v1 < b.v1)
        return true;
      if (a.v1 > b.v1)
        return false;
      return a.v2 < b.v2;
    }
  };

  explicit Mesh(const std::vector<glm::vec3> &vertices = {},
                const std::vector<Face> &faces = {});

  explicit Mesh(const std::string &ply_path);

  [[nodiscard]] const std::vector<glm::vec3> &vertices() const {
    return vertices_;
  }
  [[nodiscard]] const std::vector<Edge> &edges() const {
    return edges_;
  }
  [[nodiscard]] const std::vector<Face> &faces() const {
    return faces_;
  }

  void load_ply(const std::string &ply_path);

  void store_ply(const std::string &ply_path) const;

  void store_ply(const std::string &ply_path, const glm::vec3 &color) const;

 private:
  std::vector<glm::vec3> vertices_;
  std::vector<Edge> edges_;
  std::vector<Face> faces_;
};
