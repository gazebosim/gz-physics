ign_get_libsources_and_unittests(sources gtest_sources)

ign_add_library(${PROJECT_LIBRARY_TARGET_NAME} ${sources})

ign_set_project_public_cxx_standard(14)

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ${ignition-math${IGN_MATH_VER}_LIBRARIES}
    ignition-common${IGN_COMMON_VER})

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

ign_install_library()

ign_build_tests(
  TYPE UNIT
  SOURCES ${gtest_sources})

configure_file(utils/test_config.h.in ${PROJECT_BINARY_DIR}/utils/test_config.h)
