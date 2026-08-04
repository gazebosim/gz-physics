gz_get_sources(tests)

gz_build_tests(
  TYPE REGRESSION
  SOURCES ${tests}
  LIB_DEPS
    gz-physics-test
  ENVIRONMENT
    GZ_PHYSICS_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  TEST_LIST list)

if (BUILD_TESTING)
  foreach(test ${list})
    target_compile_definitions(${test} PRIVATE
      "MockCompositeData_LIB=\"$<TARGET_FILE:MockCompositeData>\"")
    add_dependencies(${test} MockCompositeData)
  endforeach()
endif()
