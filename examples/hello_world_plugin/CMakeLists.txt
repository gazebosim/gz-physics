cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ign-physics-hello-world-plugin)

find_package(ignition-plugin2 REQUIRED COMPONENTS all)
set(IGN_PLUGIN_VER ${ignition-plugin2_VERSION_MAJOR})

find_package(ignition-physics6 REQUIRED)
set(IGN_PHYSICS_VER ${ignition-physics6_VERSION_MAJOR})

add_library(HelloWorldPlugin SHARED HelloWorldPlugin.cc)
target_link_libraries(HelloWorldPlugin
  PRIVATE
    ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})
