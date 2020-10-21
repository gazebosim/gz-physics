cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Find the Ignition-plugin library
set(IGN_PLUGIN_VER 1)
find_package(ignition-plugin${IGN_PLUGIN_VER} 1.1 REQUIRED COMPONENTS all)

# Find the Ignition-physics library
set(IGN_PHYSICS_VER 3)
find_package(ignition-physics${IGN_PHYSICS_VER} REQUIRED)

add_executable(hello_world_loader hello_world_loader/hello_world_loader.cc)
target_link_libraries(hello_world_loader
  ignition-plugin${IGN_PLUGIN_VER}::loader
  ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})

add_library(HelloWorldPlugin SHARED hello_world_plugin/HelloWorldPlugin.cc)
target_link_libraries(HelloWorldPlugin
  PRIVATE
    ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})
