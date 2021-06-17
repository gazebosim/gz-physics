cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set(IGN_PLUGIN_VER 1)
find_package(ignition-plugin${IGN_PLUGIN_VER} 1.1 REQUIRED COMPONENTS all)

set(IGN_PHYSICS_VER 2)
find_package(ignition-physics${IGN_PHYSICS_VER} REQUIRED)

add_library(SimplePlugin SHARED plugin.cc EntityManagementFeatures.cc)
target_link_libraries(SimplePlugin
  PRIVATE
    ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})

add_executable(PluginTest EntityManagementFeatures_TEST.cc)
target_link_libraries(PluginTest
  ignition-plugin${IGN_PLUGIN_VER}::loader
  ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})

target_compile_definitions(PluginTest PRIVATE
  "simple_plugin_LIB=\"$<TARGET_FILE:SimplePlugin>\"")
