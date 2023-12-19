cmake_minimum_required(VERSION 3.22)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake)

project(mesh_decomp)

set(CMAKE_CXX_STANDARD 17)

set(ASSETS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/assets)

add_subdirectory(src)
