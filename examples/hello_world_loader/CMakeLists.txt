cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ign-physics-hello-world-loader)

find_package(gz-plugin2 REQUIRED COMPONENTS all)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-physics6 REQUIRED)
set(GZ_PHYSICS_VER ${gz-physics6_VERSION_MAJOR})

add_executable(hello_world_loader hello_world_loader.cc)
target_link_libraries(hello_world_loader
  gz-plugin${GZ_PLUGIN_VER}::loader
  gz-physics${GZ_PHYSICS_VER}::gz-physics${GZ_PHYSICS_VER})
