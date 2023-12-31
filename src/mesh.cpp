#include "mesh.h"

#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#include "fstream"
#include "map"
#include "random"
#include "set"
#include "unordered_map"

Mesh::Mesh(const std::vector<glm::vec3> &vertices,
           const std::vector<Face> &faces) {
  std::map<glm::vec3, int, vec3_lt> vertices_map;
  std::vector<glm::vec3> new_vertices;
  auto vertex_index = [&](const glm::vec3 &v) {
    if (vertices_map.count(v) == 0) {
      vertices_map[v] = new_vertices.size();
      new_vertices.push_back(v);
    }
    return vertices_map[v];
  };

  faces_.reserve(faces.size());
  for (const auto &face : faces) {
    int v1 = vertex_index(vertices[face.v1]);
    int v2 = vertex_index(vertices[face.v2]);
    int v3 = vertex_index(vertices[face.v3]);
    faces_.push_back({v1, v2, v3});
  }
  vertices_ = new_vertices;
}

Mesh::Mesh(const std::string &ply_path) {
  load_ply(ply_path);
}

void Mesh::load_ply(const std::string &ply_path) {
  Assimp::Importer importer;

  // Read the PLY file
  const aiScene *scene = importer.ReadFile(
      ply_path,
      aiProcess_Triangulate |              // Triangulate n-gons
          aiProcess_JoinIdenticalVertices  // Join identical vertices/ optimize
                                           // indexing
  );

  // If the import failed, report it
  if (!scene) {
    std::cerr << importer.GetErrorString() << std::endl;
    return;
  }

  // Now we can access the file's contents.
  std::cout << "Import of scene " << ply_path << " succeeded." << std::endl;

  // List all the meshes and show some statistics
  std::cout << "The scene contains " << scene->mNumMeshes << " meshes."
            << std::endl;

  std::vector<glm::vec3> vertices;
  std::vector<Face> faces;

  for (unsigned int n = 0; n < scene->mNumMeshes; ++n) {
    const aiMesh *mesh = scene->mMeshes[n];
    std::cout << "  Mesh " << n + 1 << ": " << mesh->mNumVertices << " vertices"
              << std::endl;
    for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
      const aiVector3D &v = mesh->mVertices[i];
      vertices.emplace_back(v.x, v.y, v.z);
    }

    for (uint32_t i = 0; i < mesh->mNumFaces; i++) {
      const aiFace &face = mesh->mFaces[i];
      if (face.mNumIndices != 3) {
        std::cerr << "Warning: face with " << face.mNumIndices << " vertices"
                  << std::endl;
        continue;
      }
      faces.emplace_back(static_cast<int>(face.mIndices[0]),
                         static_cast<int>(face.mIndices[1]),
                         static_cast<int>(face.mIndices[2]));
    }
  }

  *this = Mesh(vertices, faces);
}

void Mesh::store_ply(const std::string &ply_path) const {
  std::ofstream ply_file(ply_path);
  ply_file << "ply\n"
              "format ascii 1.0\n"
              "element vertex "
           << vertices_.size()
           << "\n"
              "property float x\n"
              "property float y\n"
              "property float z\n"
              "element face "
           << faces_.size()
           << "\n"
              "property list uchar int vertex_index\n"
              "end_header\n";
  for (const auto &v : vertices_) {
    ply_file << v.x << " " << v.y << " " << v.z << "\n";
  }
  for (const auto &f : faces_) {
    ply_file << "3 " << f.v1 << " " << f.v2 << " " << f.v3 << "\n";
  }
}

void Mesh::store_ply(const std::string &ply_path,
                     const glm::vec3 &color) const {
  std::ofstream ply_file(ply_path);
  ply_file << "ply\n"
              "format ascii 1.0\n"
              "element vertex "
           << vertices_.size()
           << "\n"
              "property float x\n"
              "property float y\n"
              "property float z\n"
              "property uchar red\n"
              "property uchar green\n"
              "property uchar blue\n"
              "element face "
           << faces_.size()
           << "\n"
              "property list uchar int vertex_index\n"
              "end_header\n";
  for (const auto &v : vertices_) {
    ply_file << v.x << " " << v.y << " " << v.z << " " << (int)(color.r * 255)
             << " " << (int)(color.g * 255) << " " << (int)(color.b * 255)
             << "\n";
  }
  for (const auto &f : faces_) {
    ply_file << "3 " << f.v1 << " " << f.v2 << " " << f.v3 << "\n";
  }
}

Graph<float> Mesh::build_dual_graph(float eta, float delta) const {
  return convert_graph(build_dual_graph(eta), delta);
}

Graph<Mesh::DualEdge> Mesh::build_dual_graph(float eta) const {
  Graph<DualEdge> origin_graph;
  std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> edge_to_faces;
  auto local_ang_dist = AngDist(eta);
  auto match_faces = [&](int m0, int m1, int v0, int face_id) {
    auto &neighbor_faces = edge_to_faces[{m1, m0}];
    for (auto &neighbor_face : neighbor_faces) {
      int v1 = neighbor_face.second;
      float geod_dist = ::geod_dist(vertices_[m0], vertices_[m1], vertices_[v0],
                                    vertices_[v1]);
      float ang_dist = local_ang_dist(vertices_[m0], vertices_[m1],
                                      vertices_[v0], vertices_[v1]);
      origin_graph.add_edge(face_id, neighbor_face.first,
                            {geod_dist, ang_dist});
      origin_graph.add_edge(neighbor_face.first, face_id,
                            {geod_dist, ang_dist});
    }
    edge_to_faces[{m0, m1}].emplace_back(face_id, v0);
  };

  for (int i = 0; i < faces_.size(); i++) {
    auto face = faces_[i];
    match_faces(face.v1, face.v2, face.v3, i);
    match_faces(face.v2, face.v3, face.v1, i);
    match_faces(face.v3, face.v1, face.v2, i);
  }

  return origin_graph;
}

Graph<float> Mesh::convert_graph(const Graph<DualEdge> &origin_graph,
                                 float delta) {
  float geod_dist_mean, ang_dist_mean;
  graph_dist_mean(origin_graph, geod_dist_mean, ang_dist_mean);
  return convert_graph(origin_graph, delta, geod_dist_mean, ang_dist_mean);
}

void Mesh::graph_dist_mean(const Graph<DualEdge> &origin_graph,
                           float &geod_dist_mean,
                           float &ang_dist_mean) {
  float geod_dist_sum = 0;
  float ang_dist_sum = 0;
  int num_edges = 0;

  for (auto &node : origin_graph.edges) {
    for (auto &edge : node.second) {
      geod_dist_sum += edge.content.geod_dist;
      ang_dist_sum += edge.content.ang_dist;
      num_edges++;
    }
  }

  geod_dist_mean = geod_dist_sum / num_edges;
  ang_dist_mean = ang_dist_sum / num_edges;
}

Graph<float> Mesh::convert_graph(const Graph<DualEdge> &origin_graph,
                                 float delta,
                                 float geod_dist_mean,
                                 float ang_dist_mean) {
  Graph<float> result;
  float max_weight = 0.0f;
  float min_weight = 1e9;
  float sum_weight = 0.0f;
  for (auto &node : origin_graph.edges) {
    for (auto &edge : node.second) {
      float geod_dist = edge.content.geod_dist / geod_dist_mean;
      float ang_dist = edge.content.ang_dist / ang_dist_mean;
      float weight = delta * geod_dist + (1.0f - delta) * ang_dist;
      result.add_edge(node.first, edge.dst, weight);
      max_weight = std::max(max_weight, weight);
      min_weight = std::min(min_weight, weight);
      sum_weight += weight;
    }
  }

  printf("max_weight: %f\n", max_weight);
  printf("min_weight: %f\n", min_weight);
  printf("sum_weight: %f\n", sum_weight);

  return result;
}

Graph<float> Mesh::build_ang_graph() const {
  Graph<float> graph;
  std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> edge_to_faces;
  auto match_faces = [&](int m0, int m1, int v0, int face_id) {
    auto &neighbor_faces = edge_to_faces[{m1, m0}];
    for (auto &neighbor_face : neighbor_faces) {
      int v1 = neighbor_face.second;
      float ang = dual_face_ang(vertices_[m0], vertices_[m1], vertices_[v0],
                                vertices_[v1]);
      graph.add_edge(face_id, neighbor_face.first, ang);
      graph.add_edge(neighbor_face.first, face_id, ang);
    }
    edge_to_faces[{m0, m1}].emplace_back(face_id, v0);
  };

  for (int i = 0; i < faces_.size(); i++) {
    auto face = faces_[i];
    match_faces(face.v1, face.v2, face.v3, i);
    match_faces(face.v2, face.v3, face.v1, i);
    match_faces(face.v3, face.v1, face.v2, i);
  }

  return graph;
}

void MeshAssembler::AddFace(const glm::vec3 &v1,
                            const glm::vec3 &v2,
                            const glm::vec3 &v3) {
  auto vertex_index = [&](const glm::vec3 &v) {
    if (vertex_to_index_.count(v) == 0) {
      vertex_to_index_[v] = vertices_.size();
      vertices_.push_back(v);
    }
    return vertex_to_index_[v];
  };
  faces_.emplace_back(vertex_index(v1), vertex_index(v2), vertex_index(v3));
}

Mesh MeshAssembler::GetMesh() const {
  return Mesh{vertices_, faces_};
}

void store_combined_ply(const std::string &ply_path,
                        const std::vector<Mesh> &meshes) {
  std::ofstream ply_file(ply_path);

  int num_vertices = 0;
  int num_faces = 0;

  for (const auto &mesh : meshes) {
    num_vertices += mesh.vertices().size();
    num_faces += mesh.faces().size();
  }

  ply_file << "ply\n"
              "format ascii 1.0\n"
              "element vertex "
           << num_vertices
           << "\n"
              "property float x\n"
              "property float y\n"
              "property float z\n"
              "property uchar red\n"
              "property uchar green\n"
              "property uchar blue\n"
              "element face "
           << num_faces
           << "\n"
              "property list uchar int vertex_index\n"
              "end_header\n";

  std::mt19937 rng(0);

  for (const auto &mesh : meshes) {
    glm::vec3 color =
        glm::vec3(std::uniform_real_distribution<float>(0.0f, 1.0f)(rng),
                  std::uniform_real_distribution<float>(0.0f, 1.0f)(rng),
                  std::uniform_real_distribution<float>(0.0f, 1.0f)(rng));
    for (const auto &v : mesh.vertices()) {
      ply_file << v.x << " " << v.y << " " << v.z << " " << (int)(color.r * 255)
               << " " << (int)(color.g * 255) << " " << (int)(color.b * 255)
               << "\n";
    }
  }

  int offset = 0;
  for (const auto &mesh : meshes) {
    for (const auto &f : mesh.faces()) {
      ply_file << "3 " << f.v1 + offset << " " << f.v2 + offset << " "
               << f.v3 + offset << "\n";
    }
    offset += mesh.vertices().size();
  }

  ply_file.close();
}
