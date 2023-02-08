set(TEST_TYPE "COMMON_TEST")

set(tests
  added_mass
  addexternalforcetorque
  basic_test
  collisions
  construct_empty_world
  detachable_joint
  free_joint_features
  joint_features
  joint_transmitted_wrench_features
  kinematic_features
  link_features
  shape_features
  simulation_features
  world_features
)

set(TEST_INSTALL_DIR ${CMAKE_INSTALL_LIBEXECDIR}/gz/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}/)

# Find the Python interpreter for running the
# check_test_ran.py script
include(GzPython)

function(configure_common_test PHYSICS_ENGINE_NAME test_name)
  if(NOT SKIP_${PHYSICS_ENGINE_NAME} AND NOT INTERNAL_SKIP_${PHYSICS_ENGINE_NAME})
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
      gz-common${GZ_COMMON_VER}::testing
      ${PROJECT_LIBRARY_TARGET_NAME}
      ${PROJECT_LIBRARY_TARGET_NAME}-sdf
      ${PROJECT_LIBRARY_TARGET_NAME}-mesh
      gtest
      gtest_main
  )

  target_include_directories(${test_executable}
    PUBLIC
    ${PROJECT_SOURCE_DIR}/test)

  target_compile_definitions(${test_executable} PRIVATE
    "GZ_PHYSICS_RESOURCE_DIR=\"${GZ_PHYSICS_RESOURCE_DIR}\""
    "TESTING_PROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\""
  )

  install(TARGETS ${test_executable} DESTINATION ${TEST_INSTALL_DIR})

  configure_common_test("bullet" ${test_executable})
  configure_common_test("bullet-featherstone" ${test_executable})
  configure_common_test("dartsim" ${test_executable})
  configure_common_test("tpe" ${test_executable})
endforeach()

if(TARGET COMMON_TEST_joint_transmitted_wrench_features)
  target_include_directories(COMMON_TEST_joint_transmitted_wrench_features PRIVATE ${BULLET_INCLUDE_DIRS})
endif()
