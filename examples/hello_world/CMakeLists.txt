cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Find the Ignition-Math library
set(IGN_PHYSICS_VER 2)
find_package(ignition-physics${IGN_PHYSICS_VER} REQUIRED)

add_library(HelloWorldPlugin HelloWorldPlugin.cc)
target_link_libraries(HelloWorldPlugin
  PRIVATE
    ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})

