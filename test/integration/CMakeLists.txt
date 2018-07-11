
ign_get_sources(tests)

ign_build_tests(
  TYPE INTEGRATION
  SOURCES ${tests}
  LIB_DEPS Eigen3::Eigen)

target_compile_definitions(INTEGRATION_FeatureSystem PRIVATE
  "DETAIL_IGN_MOCK_PLUGIN_PATH=\"$<TARGET_FILE_DIR:MockEntities>\"")

add_dependencies(INTEGRATION_FeatureSystem MockEntities)
