
ign_get_sources(tests)

ign_build_tests(
  TYPE INTEGRATION
  SOURCES ${tests}
  LIB_DEPS
    Eigen3::Eigen
    ignition-plugin${IGN_PLUGIN_VER}::loader
  TEST_LIST list)

if (BUILD_TESTING)
  foreach(test ${list})

    foreach(mock_plugin
        MockDoublePendulum
        MockEntities
        MockFrames
        MockJoints)

      target_compile_definitions(${test} PRIVATE
        "${mock_plugin}_LIB=\"$<TARGET_FILE:${mock_plugin}>\"")

      add_dependencies(${test} ${mock_plugin})

    endforeach()

  endforeach()
endif()
