cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-physics-hello-world-loader)

find_package(gz-plugin REQUIRED COMPONENTS all)

find_package(gz-physics REQUIRED)

add_executable(hello_world_loader hello_world_loader.cc)
target_link_libraries(hello_world_loader
  gz-plugin::loader
  gz-physics::gz-physics)
