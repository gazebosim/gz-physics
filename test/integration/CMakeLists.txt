
ign_get_sources(tests)

ign_build_tests(
  TYPE INTEGRATION
  SOURCES ${tests}
  LIB_DEPS
    Eigen3::Eigen
    ignition-plugin${IGN_PLUGIN_VER}::loader)

if (BUILD_TESTING)
  target_compile_definitions(INTEGRATION_FeatureSystem PRIVATE
    "MockEntities_LIB=\"$<TARGET_FILE:MockEntities>\"")

  add_dependencies(INTEGRATION_FeatureSystem MockEntities)
endif()
