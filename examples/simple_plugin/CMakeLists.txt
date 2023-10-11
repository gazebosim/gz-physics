cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(gz-physics-simple-plugin)

find_package(gz-plugin2 REQUIRED COMPONENTS all)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-physics8 REQUIRED)
set(GZ_PHYSICS_VER ${gz-physics8_VERSION_MAJOR})

add_library(SimplePlugin SHARED plugin.cc EntityManagementFeatures.cc)
target_link_libraries(SimplePlugin
  PRIVATE
    gz-physics${GZ_PHYSICS_VER}::gz-physics${GZ_PHYSICS_VER})

add_executable(PluginTest EntityManagementFeatures_TEST.cc)
target_link_libraries(PluginTest
  gz-plugin${GZ_PLUGIN_VER}::loader
  gz-physics${GZ_PHYSICS_VER}::gz-physics${GZ_PHYSICS_VER})

target_compile_definitions(PluginTest PRIVATE
  "simple_plugin_LIB=\"$<TARGET_FILE:SimplePlugin>\"")
