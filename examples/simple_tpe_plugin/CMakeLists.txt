cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set(engine_name tpe-plugin)
ign_add_component(${engine_name}
  SOURCES ${sources}
  DEPENDS_ON_COMPONENTS tpe
  GET_TARGET_NAME tpe_plugin)

set(tpelib_dir ${PROJECT_SOURCE_DIR}/tpe)
target_include_directories(${tpe_plugin} PRIVATE ${tpelib_dir})

set(IGN_PLUGIN_VER 1)
find_package(ignition-plugin${IGN_PLUGIN_VER} 1.1 REQUIRED COMPONENTS all)

set(IGN_PHYSICS_VER 2)
find_package(ignition-physics${IGN_PHYSICS_VER} REQUIRED)

add_library(SimpleTPEPLugin SHARED plugin.cc)
target_link_libraries(SimpleTPEPlugin
  PRIVATE
    ignition-physics${IGN_PHYSICS_VER}::ignition-physics${IGN_PHYSICS_VER})
