file(GLOB SOURCES *.cpp)

add_executable(mesh_decomp ${SOURCES})

find_package(glm CONFIG REQUIRED)
target_link_libraries(mesh_decomp PRIVATE glm::glm)

find_package(assimp CONFIG REQUIRED)
target_link_libraries(mesh_decomp PRIVATE assimp::assimp)
