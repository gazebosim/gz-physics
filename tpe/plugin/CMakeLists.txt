# This component expresses custom features of the tpesim plugin, which can
# expose native tpesim data types.
ign_add_component(tpesim INTERFACE
  DEPENDS_ON_COMPONENTS sdf mesh
  GET_TARGET_NAME features)

target_link_libraries(${features} INTERFACE ${PROJECT_LIBRARY_TARGET_NAME}-tpe)
target_include_directories(${features} SYSTEM INTERFACE)

install(
  DIRECTORY include/
  DESTINATION "${IGN_INCLUDE_INSTALL_DIR_FULL}")

ign_get_libsources_and_unittests(sources test_sources)

# TODO(MXG): Think about an ign_add_plugin(~) macro for ign-cmake
ign_add_component(tpesim-plugin
  SOURCES ${sources}
  DEPENDS_ON_COMPONENTS tpesim
  GET_TARGET_NAME tpesim_plugin)

target_link_libraries(${tpesim_plugin}
  PUBLIC
    ${features}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-math${IGN_MATH_VER}::eigen3)

# We need to link this, even when the profiler isn't used to get headers.
target_link_libraries(${tpesim_plugin}
  PRIVATE
  ignition-common${IGN_COMMON_VER}::profiler
)

ign_build_tests(
  TYPE UNIT_tpe
  SOURCES ${test_sources}
  LIB_DEPS
    ${features}
    ignition-plugin${IGN_PLUGIN_VER}::loader
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
  TEST_LIST tests)

foreach(test ${tests})

  target_compile_definitions(${test} PRIVATE
    "tpesim_plugin_LIB=\"$<TARGET_FILE:${tpesim_plugin}>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/worlds/\""
    "IGNITION_PHYSICS_RESOURCE_DIR=\"${IGNITION_PHYSICS_RESOURCE_DIR}\"")

endforeach()


