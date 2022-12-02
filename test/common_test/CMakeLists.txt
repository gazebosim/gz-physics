set(TEST_TYPE "COMMON_TEST")

set(tests
  addexternalforcetorque
  basic_test
  collisions
  construct_empty_world
  free_joint_features
  joint_features
  kinematic_features
  link_features
  shape_features
  simulation_features
  world_features
)

function(configure_common_test PHYSICS_ENGINE_NAME test_name)
  add_test(NAME ${test_name}_${PHYSICS_ENGINE_NAME}
    COMMAND
      ${test_name}
      $<TARGET_FILE:${PROJECT_LIBRARY_TARGET_NAME}-${PHYSICS_ENGINE_NAME}-plugin>
  )
  set(_env_vars)
  list(APPEND _env_vars "GZ_PHYSICS_RESOURCE_DIR=${GZ_PHYSICS_RESOURCE_DIR}")
  set_tests_properties(${test_name}_${PHYSICS_ENGINE_NAME} PROPERTIES
    ENVIRONMENT "${_env_vars}")
endfunction()

set(GZ_PHYSICS_RESOURCE_DIR "${CMAKE_SOURCE_DIR}/resources")

foreach(test ${tests})
  add_executable(${TEST_TYPE}_${test} ${test}.cc)

  target_link_libraries(${TEST_TYPE}_${test}
    PUBLIC
      gz-plugin${GZ_PLUGIN_VER}::loader
      gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
      ${PROJECT_LIBRARY_TARGET_NAME}
      ${PROJECT_LIBRARY_TARGET_NAME}-sdf
      ${PROJECT_LIBRARY_TARGET_NAME}-mesh
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
