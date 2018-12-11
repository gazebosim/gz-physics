
ign_get_sources(tests)

if (NOT DART_FOUND)
  list(REMOVE_ITEM tests DoublePendulum.cc)
endif()

ign_build_tests(
  TYPE INTEGRATION
  SOURCES ${tests}
  LIB_DEPS
    Eigen3::Eigen
    ignition-plugin${IGN_PLUGIN_VER}::loader
  TEST_LIST list)

if (BUILD_TESTING)
  foreach(test ${list})

    list(APPEND mock_plugins
        MockEntities
        MockFrames
        MockJoints)
    if (DART_FOUND)
      list(APPEND plugins MockDoublePendulum)
    endif()

    foreach(mock_plugin IN LISTS mock_plugins)

      target_compile_definitions(${test} PRIVATE
        "${mock_plugin}_LIB=\"$<TARGET_FILE:${mock_plugin}>\"")

      add_dependencies(${test} ${mock_plugin})

    endforeach()

  endforeach()
endif()
