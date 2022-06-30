gz_get_libsources_and_unittests(sources gtest_sources)
gz_create_core_library(SOURCES ${sources}) 


target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
    gz-plugin${GZ_PLUGIN_VER}::register
    Eigen3::Eigen)

gz_build_tests(
  TYPE UNIT
  SOURCES ${gtest_sources})

if(TARGET UNIT_FindFeatures_TEST)
  target_link_libraries(UNIT_FindFeatures_TEST
    gz-plugin${GZ_PLUGIN_VER}::loader)
endif()
