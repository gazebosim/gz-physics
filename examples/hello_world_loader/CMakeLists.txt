cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ign-physics-hello-world-loader)

find_package(ignition-plugin2 REQUIRED COMPONENTS all)
set(IGN_PLUGIN_VER ${ignition-plugin2_VERSION_MAJOR})

find_package(ignition-physics6 REQUIRED)
set(IGN_PHYSICS_VER ${ignition-physics6_VERSION_MAJOR})

add_executable(hello_world_loader hello_world_loader.cc)
target_link_libraries(hello_world_loader
  ignition-plugin${IGN_PLUGIN_VER}::loader
  ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})
