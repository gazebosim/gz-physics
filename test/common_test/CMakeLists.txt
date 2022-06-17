set(TEST_TYPE "COMMON_TEST")

set(tests
  basic_test
)

link_directories(${PROJECT_BINARY_DIR}/test)

function(configure_common_test PHYSICS_ENGINE_NAME test_name)
  add_test(NAME ${test_name}_${PHYSICS_ENGINE_NAME}
    COMMAND
      ${test_name}
      ${CMAKE_BINARY_DIR}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_LIBRARY_TARGET_NAME}-${PHYSICS_ENGINE_NAME}-plugin${CMAKE_SHARED_LIBRARY_SUFFIX}
  )
endfunction()

foreach(test ${tests})
  add_executable(${TEST_TYPE}_${test} ${test}.cc)

  target_link_libraries(${TEST_TYPE}_${test}
    PUBLIC
      ignition-plugin${IGN_PLUGIN_VER}::loader
      ignition-common5::ignition-common5
      ${PROJECT_LIBRARY_TARGET_NAME}
      gtest
      gtest_main
  )

  if (${BULLET_FOUND})
    configure_common_test("bullet" ${TEST_TYPE}_${test})
  endif()
  if (${DART_FOUND})
    configure_common_test("dartsim" ${TEST_TYPE}_${test})
  endif()
  configure_common_test("tpe" ${TEST_TYPE}_${test})
endforeach()
