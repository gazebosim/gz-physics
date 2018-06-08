
ign_get_sources(tests)

ign_build_tests(
  TYPE INTEGRATION
  SOURCES ${tests})

target_compile_definitions(INTEGRATION_FeatureSystem PRIVATE
  "DETAIL_IGN_MOCK_PLUGIN_PATH=\"$<TARGET_FILE_DIR:MockPhysics>\"")

add_dependencies(INTEGRATION_FeatureSystem MockPhysics)
