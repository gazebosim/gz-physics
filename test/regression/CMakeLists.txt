gz_get_sources(tests)

gz_build_tests(
  TYPE REGRESSION
  SOURCES ${tests}
  LIB_DEPS
    gz-physics-test
  ENVIRONMENT
    GZ_PHYSICS_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  TEST_LIST list)

if(TARGET REGRESSION_1019_type_index_dso)
  target_compile_definitions(REGRESSION_1019_type_index_dso PRIVATE
    "MockCompositeData_LIB=\"$<TARGET_FILE:MockCompositeData>\"")
  add_dependencies(REGRESSION_1019_type_index_dso MockCompositeData)
endif()
