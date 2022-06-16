gz_get_libsources_and_unittests(sources gtest_sources)

gz_create_core_library(SOURCES ${sources} CXX_STANDARD 17)

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ${ignition-math${IGN_MATH_VER}_LIBRARIES}
    ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
    ignition-plugin${IGN_PLUGIN_VER}::register
    Eigen3::Eigen)

gz_build_tests(
  TYPE UNIT
  SOURCES ${gtest_sources})

if(TARGET UNIT_FindFeatures_TEST)
  target_link_libraries(UNIT_FindFeatures_TEST
    ignition-plugin${IGN_PLUGIN_VER}::loader)
endif()
