ign_get_libsources_and_unittests(sources test_sources)

ign_add_component(tpe-lib
  SOURCES ${sources}
  GET_TARGET_NAME tpe_lib_target
  CXX_STANDARD 17)

target_link_libraries(${tpe_lib_target}
  PUBLIC
  PRIVATE
    ignition-common${IGN_COMMON_VER}::graphics
)

ign_build_tests(
  TYPE UNIT_tpe_lib
  SOURCES ${test_sources}
  LIB_DEPS
    ${features}
    ignition-plugin${IGN_PLUGIN_VER}::loader
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    ${PROJECT_LIBRARY_TARGET_NAME}-tpe-lib
  TEST_LIST tests)
