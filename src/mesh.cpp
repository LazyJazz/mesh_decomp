#include "mesh.h"

#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#include "fstream"
#include "map"
#include "set"
#include "unordered_map"

Mesh::Mesh(const std::vector<glm::vec3> &vertices,
           const std::vector<Face> &faces) {
  std::map<glm::vec3, int, vec3_lt> vertices_map;
  std::set<Edge> edges_set;
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
    edges_set.insert({v1, v2});
    edges_set.insert({v2, v3});
    edges_set.insert({v3, v1});
    faces_.push_back({v1, v2, v3});
  }
  edges_.reserve(edges_set.size());
  for (const auto &edge : edges_set) {
    edges_.push_back(edge);
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
