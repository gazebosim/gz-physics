ign_get_libsources_and_unittests(sources test_sources)

ign_add_component(tpelib
  SOURCES ${sources}
  GET_TARGET_NAME tpelib_target
  CXX_STANDARD 17)

target_link_libraries(${tpelib_target}
  PUBLIC
  PRIVATE
    ignition-common${IGN_COMMON_VER}::requested
    # ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    # ignition-common${IGN_COMMON_VER}::graphics
)
 ign_build_tests(
  TYPE UNIT_tpelib
  SOURCES ${test_sources}
  LIB_DEPS
    ${features}
    ignition-plugin${IGN_PLUGIN_VER}::loader
    ignition-common${IGN_COMMON_VER}::requested
    # ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    # ignition-common${IGN_COMMON_VER}::graphics
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    ${tpelib_target}
  TEST_LIST tests)
