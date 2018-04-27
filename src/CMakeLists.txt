ign_get_libsources_and_unittests(sources gtest_sources)

ign_create_core_library(SOURCES ${sources})

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ${ignition-math${IGN_MATH_VER}_LIBRARIES}
    ${ignition-common${IGN_COMMON_VER}_LIBRARIES})

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

ign_build_tests(
  TYPE UNIT
  SOURCES ${gtest_sources})

if(MSVC)
  if(TARGET UNIT_SpecifyData_TEST)
    target_compile_options(UNIT_SpecifyData_TEST PRIVATE /permissive-)
  endif()
endif()
