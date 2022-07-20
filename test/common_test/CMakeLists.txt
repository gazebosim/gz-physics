set(TEST_TYPE "COMMON_TEST")

set(tests
  basic_test
  construct_empty_world
  free_joint_features
  simulation_features
  world_features
)

function(configure_common_test PHYSICS_ENGINE_NAME test_name)
  add_test(NAME ${test_name}_${PHYSICS_ENGINE_NAME}
    COMMAND
      ${test_name}
      $<TARGET_FILE:${PROJECT_LIBRARY_TARGET_NAME}-${PHYSICS_ENGINE_NAME}-plugin>
  )
endfunction()

foreach(test ${tests})
  add_executable(${TEST_TYPE}_${test} ${test}.cc)

  target_link_libraries(${TEST_TYPE}_${test}
    PUBLIC
      gz-plugin${GZ_PLUGIN_VER}::loader
      gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
      ${PROJECT_LIBRARY_TARGET_NAME}
      ${PROJECT_LIBRARY_TARGET_NAME}-sdf
      gtest
      gtest_main
      ${PROJECT_NAME}_test_lib_loader
  )

  target_compile_definitions(${TEST_TYPE}_${test} PRIVATE
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/worlds/\""
  )

  if (${BULLET_FOUND})
    configure_common_test("bullet" ${TEST_TYPE}_${test})
    configure_common_test("bullet-featherstone" ${TEST_TYPE}_${test})
  endif()
  if (${DART_FOUND})
    configure_common_test("dartsim" ${TEST_TYPE}_${test})
  endif()
  configure_common_test("tpe" ${TEST_TYPE}_${test})
endforeach()
