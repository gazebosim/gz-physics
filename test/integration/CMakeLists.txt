
ign_get_sources(tests)

ign_build_tests(
  TYPE INTEGRATION
  SOURCES ${tests}
  LIB_DEPS Eigen3::Eigen
  TEST_LIST list)

foreach(test ${list})
  target_compile_definitions(${test} PRIVATE
    "DETAIL_IGN_MOCK_PLUGIN_PATH=\"$<TARGET_FILE_DIR:MockPhysics>\"")
endforeach()

add_dependencies(INTEGRATION_FeatureSystem MockPhysics)
