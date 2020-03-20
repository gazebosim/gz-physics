ign_get_libsources_and_unittests(sources test_sources)

ign_add_component(tpeengine
  SOURCES ${sources}
  GET_TARGET_NAME tpeengine_target
  CXX_STANDARD 17)

target_link_libraries(${tpeengine_target}
  PUBLIC
  PRIVATE
    ignition-common${IGN_COMMON_VER}::requested
    # ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    # ignition-common${IGN_COMMON_VER}::graphics
)
 ign_build_tests(
  TYPE UNIT_tpeengine
  SOURCES ${test_sources}
  LIB_DEPS
    ${features}
    ignition-plugin${IGN_PLUGIN_VER}::loader
    ignition-common${IGN_COMMON_VER}::requested
    # ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    # ignition-common${IGN_COMMON_VER}::graphics
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    ${tpeengine_target}
  TEST_LIST tests)
