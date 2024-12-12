cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-physics-hello-world-loader)

find_package(gz-plugin3 REQUIRED COMPONENTS all)
set(GZ_PLUGIN_VER ${gz-plugin3_VERSION_MAJOR})

find_package(gz-physics8 REQUIRED)
set(GZ_PHYSICS_VER ${gz-physics8_VERSION_MAJOR})

add_executable(hello_world_loader hello_world_loader.cc)
target_link_libraries(hello_world_loader
  gz-plugin${GZ_PLUGIN_VER}::loader
  gz-physics${GZ_PHYSICS_VER}::gz-physics${GZ_PHYSICS_VER})
