cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set(IGN_PLUGIN_VER 2)
find_package(ignition-plugin${IGN_PLUGIN_VER} REQUIRED COMPONENTS all)

set(IGN_PHYSICS_VER 4)
find_package(ignition-physics${IGN_PHYSICS_VER} REQUIRED)

add_executable(hello_world_loader hello_world_loader.cc)
target_link_libraries(hello_world_loader
  ignition-plugin${IGN_PLUGIN_VER}::loader
  ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})
