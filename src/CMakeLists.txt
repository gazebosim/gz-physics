ign_get_libsources_and_unittests(sources gtest_sources)

ign_create_core_library(SOURCES ${sources})

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ${ignition-math${IGN_MATH_VER}_LIBRARIES}
    ${ignition-common${IGN_COMMON_VER}_LIBRARIES})

ign_build_tests(
  TYPE UNIT
  SOURCES ${gtest_sources})

if(MSVC)
  if(TARGET UNIT_SpecifyData_TEST)
    target_compile_options(UNIT_SpecifyData_TEST PRIVATE /permissive-)
  endif()
endif()

configure_file(utils/test_config.h.in ${PROJECT_BINARY_DIR}/utils/test_config.h)
