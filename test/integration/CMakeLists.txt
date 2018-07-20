
ign_get_sources(tests)

ign_build_tests(
  TYPE INTEGRATION
  SOURCES ${tests}
  LIB_DEPS
    Eigen3::Eigen
    ignition-plugin${IGN_PLUGIN_VER}::loader
  TEST_LIST list)

foreach(test ${list})
  target_compile_definitions(INTEGRATION_FeatureSystem PRIVATE
    "MockEntities_LIB=\"$<TARGET_FILE:MockEntities>\"")
endforeach()

add_dependencies(INTEGRATION_FeatureSystem MockEntities)
