set(TEST_TYPE "COMMON_TEST")

set(tests
  added_mass
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

# Find the Python interpreter for running the
# check_test_ran.py script
include(GzPython)

function(configure_common_test PHYSICS_ENGINE_NAME test_name)
  set(target_name ${test_name}_${PHYSICS_ENGINE_NAME})
  add_test(NAME ${target_name}
    COMMAND
      ${test_name}
      $<TARGET_FILE:${PROJECT_LIBRARY_TARGET_NAME}-${PHYSICS_ENGINE_NAME}-plugin>
      --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${target_name}.xml
  )

  set_tests_properties(${target_name} PROPERTIES TIMEOUT 240)

  if(Python3_Interpreter_FOUND)
    # Check that the test produced a result and create a failure if it didn't.
    # Guards against crashed and timed out tests.
    add_test(check_${target_name} ${Python3_EXECUTABLE} ${GZ_CMAKE_TOOLS_DIR}/check_test_ran.py
      ${CMAKE_BINARY_DIR}/test_results/${target_name}.xml)
  endif()
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
