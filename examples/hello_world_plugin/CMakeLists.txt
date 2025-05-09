cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-physics-hello-world-plugin)

find_package(gz-plugin REQUIRED COMPONENTS all)

find_package(gz-physics9 REQUIRED)
set(GZ_PHYSICS_VER ${gz-physics9_VERSION_MAJOR})

add_library(HelloWorldPlugin SHARED HelloWorldPlugin.cc)
target_link_libraries(HelloWorldPlugin
  PRIVATE
    gz-physics::gz-physics)
