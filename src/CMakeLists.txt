file(GLOB SOURCES *.cpp)

add_executable(mesh_decomp ${SOURCES})

find_package(glm CONFIG REQUIRED)
target_link_libraries(mesh_decomp PRIVATE glm::glm)

find_package(assimp CONFIG REQUIRED)
target_link_libraries(mesh_decomp PRIVATE assimp::assimp)

# Add def of ASSETS_DIR to mesh_decomp
target_compile_definitions(mesh_decomp PRIVATE ASSETS_DIR="${ASSETS_DIR}/")
