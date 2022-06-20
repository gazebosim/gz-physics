gz_get_sources(assert_tests)

if(UNIX)

  foreach(test_file ${assert_tests})
    string(REGEX REPLACE ".cc" "" test ${test_file})
    add_executable(${test} ${test_file})
    target_link_libraries(${test} ${PROJECT_LIBRARY_TARGET_NAME})
    set_target_properties(${test} PROPERTIES
      EXCLUDE_FROM_ALL TRUE
      EXCLUDE_FROM_DEFAULT_BUILD TRUE)
    add_test(NAME STATIC_ASSERT_${test}
      COMMAND ${CMAKE_SOURCE_DIR}/test/static_assert/testrunner.bash ${CMAKE_COMMAND} --build . --target ${test} --config $<CONFIGURATION>
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
  endforeach()

  set_tests_properties(STATIC_ASSERT_FeatureList_conflict PROPERTIES
    PASS_REGULAR_EXPRESSION "FEATURE CONFLICT DETECTED")

  set_tests_properties(STATIC_ASSERT_FeatureList_nested_conflict PROPERTIES
    PASS_REGULAR_EXPRESSION "FEATURE CONFLICT DETECTED")

  set_tests_properties(STATIC_ASSERT_FeatureList_InvalidFeature PROPERTIES
    PASS_REGULAR_EXPRESSION "NOT A Feature")

endif()
