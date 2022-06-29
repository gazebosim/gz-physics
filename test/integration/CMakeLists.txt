gz_get_sources(tests)

if (NOT DART_FOUND)
  list(REMOVE_ITEM tests DoublePendulum.cc)
endif()

configure_file (test_config.h.in ${PROJECT_BINARY_DIR}/test_config.h)

gz_build_tests(
  TYPE INTEGRATION
  SOURCES ${tests}
  LIB_DEPS
    Eigen3::Eigen
    gz-plugin${GZ_PLUGIN_VER}::loader
  TEST_LIST list)

if (BUILD_TESTING)
  foreach(test ${list})

    list(APPEND mock_plugins
        MockEntities
        MockFrames
        MockJoints)
    if (DART_FOUND)
      list(APPEND mock_plugins MockDoublePendulum)
    endif()

    foreach(mock_plugin IN LISTS mock_plugins)

      target_compile_definitions(${test} PRIVATE
        "${mock_plugin}_LIB=\"$<TARGET_FILE:${mock_plugin}>\"")

      add_dependencies(${test} ${mock_plugin})

    endforeach()

  endforeach()
endif()

if(TARGET INTEGRATION_ExamplesBuild_TEST)
  add_dependencies(INTEGRATION_ExamplesBuild_TEST FAKE_INSTALL)
  set(_env_vars)
  list(APPEND _env_vars "CMAKE_PREFIX_PATH=${FAKE_INSTALL_PREFIX}:$ENV{CMAKE_PREFIX_PATH}")
  set_tests_properties(INTEGRATION_ExamplesBuild_TEST PROPERTIES
    ENVIRONMENT "${_env_vars}")
endif()
