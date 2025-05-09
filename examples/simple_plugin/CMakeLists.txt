cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

project(gz-physics-simple-plugin)

find_package(gz-plugin REQUIRED COMPONENTS all)

find_package(gz-physics9 REQUIRED)
set(GZ_PHYSICS_VER ${gz-physics9_VERSION_MAJOR})

add_library(SimplePlugin SHARED plugin.cc EntityManagementFeatures.cc)
target_link_libraries(SimplePlugin
  PRIVATE
    gz-physics::gz-physics)

add_executable(PluginTest EntityManagementFeatures_TEST.cc)
target_link_libraries(PluginTest
  gz-plugin::loader
  gz-physics::gz-physics)

target_compile_definitions(PluginTest PRIVATE
  "simple_plugin_LIB=\"$<TARGET_FILE:SimplePlugin>\"")
