cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(gz-physics-hello-world-plugin)

find_package(gz-plugin2 REQUIRED COMPONENTS all)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-physics7 REQUIRED)
set(GZ_PHYSICS_VER ${gz-physics7_VERSION_MAJOR})

add_library(HelloWorldPlugin SHARED HelloWorldPlugin.cc)
target_link_libraries(HelloWorldPlugin
  PRIVATE
    gz-physics${GZ_PHYSICS_VER}::gz-physics${GZ_PHYSICS_VER})
