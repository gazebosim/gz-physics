cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ign-physics-hello-world-plugin)

set(IGN_PLUGIN_VER 1)
find_package(ignition-plugin${IGN_PLUGIN_VER} 1.1 REQUIRED COMPONENTS all)

set(IGN_PHYSICS_VER 6)
find_package(ignition-physics${IGN_PHYSICS_VER} REQUIRED)

add_library(HelloWorldPlugin SHARED HelloWorldPlugin.cc)
target_link_libraries(HelloWorldPlugin
  PRIVATE
    ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})
