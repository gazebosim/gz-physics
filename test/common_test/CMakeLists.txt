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
endfunction()

set(GZ_PHYSICS_RESOURCE_DIR "${CMAKE_SOURCE_DIR}/resources")

foreach(test ${tests})
  set(test_executable "${TEST_TYPE}_${test}")
  add_executable(${test_executable} ${test}.cc)

  target_link_libraries(${test_executable}
    PUBLIC
      gz-plugin${GZ_PLUGIN_VER}::loader
      gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
      ${PROJECT_LIBRARY_TARGET_NAME}
      ${PROJECT_LIBRARY_TARGET_NAME}-sdf
      ${PROJECT_LIBRARY_TARGET_NAME}-mesh
      gtest
      gtest_main
  )

  target_compile_definitions(${test_executable} PRIVATE
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/worlds/\""
    "GZ_PHYSICS_RESOURCE_DIR=\"${GZ_PHYSICS_RESOURCE_DIR}\""
  )

  if (${BULLET_FOUND})
    configure_common_test("bullet" ${test_executable})
    configure_common_test("bullet-featherstone" ${test_executable})
  endif()
  if (${DART_FOUND})
    configure_common_test("dartsim" ${test_executable})
  endif()
  configure_common_test("tpe" ${test_executable})
endforeach()
