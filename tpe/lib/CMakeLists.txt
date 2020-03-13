install(
  DIRECTORY include/
  DESTINATION "${IGN_INCLUDE_INSTALL_DIR_FULL}")

ign_get_libsources_and_unittests(sources test_sources)

ign_add_component(tpe
  SOURCES ${sources}
  GET_TARGET_NAME tpe_target
  CXX_STANDARD 17)

target_link_libraries(${tpe_target}
  PUBLIC
  PRIVATE
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    # ignition-plugin${IGN_PLUGIN_VER}::register
)

target_include_directories(${tpe_target}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
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
    ${PROJECT_LIBRARY_TARGET_NAME}-tpe
  TEST_LIST tests)
