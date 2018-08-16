ign_get_libsources_and_unittests(sources gtest_sources)

ign_create_core_library(SOURCES ${sources} CXX_STANDARD 17)

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ${ignition-math${IGN_MATH_VER}_LIBRARIES}
#    ${ignition-common${IGN_COMMON_VER}_LIBRARIES}
    ignition-plugin${IGN_PLUGIN_VER}::register)

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

ign_build_tests(
  TYPE UNIT
  SOURCES ${gtest_sources})
