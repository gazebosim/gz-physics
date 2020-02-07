cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

# Find the Ignition-Math library
set(IGN_PLUGIN_VER 1)
find_package(ignition-plugin${IGN_PLUGIN_VER} 1.1 REQUIRED COMPONENTS all)

set(IGN_PHYSICS_VER 2)
find_package(ignition-physics${IGN_PHYSICS_VER} REQUIRED)

add_library(HelloWorldPlugin SHARED HelloWorldPlugin.cc)
target_link_libraries(HelloWorldPlugin
  PRIVATE
    ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})

add_executable(hello_world hello_world.cc)
target_link_libraries(hello_world
  ignition-plugin${IGN_PLUGIN_VER}::loader
  ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})
target_compile_definitions(hello_world PRIVATE
  "HelloWorldPlugin_LIB=\"$<TARGET_FILE:HelloWorldPlugin>\"")
add_dependencies(hello_world HelloWorldPlugin)
