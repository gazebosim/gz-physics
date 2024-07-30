set(TEST_TYPE "COMMON_TEST")
include_directories(${BULLET_INCLUDE_DIRS})

set(tests
  added_mass
  addexternalforcetorque
  basic_test
  collisions
  construct_empty_world
  detachable_joint
  free_joint_features
  joint_features
  joint_mimic_features
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
find_package(Python3 REQUIRED COMPONENTS Interpreter)

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

# Get bullet version using pkg_check_modules as it is not available
# through the cmake module
gz_pkg_check_modules_quiet(bullet_version_check "bullet")

foreach(test ${tests})
  set(test_executable "${TEST_TYPE}_${test}")
  add_executable(${test_executable} ${test}.cc)

  target_link_libraries(${test_executable}
    PUBLIC
      gz-physics-test
      ${PROJECT_LIBRARY_TARGET_NAME}
      ${PROJECT_LIBRARY_TARGET_NAME}-sdf
      ${PROJECT_LIBRARY_TARGET_NAME}-mesh
      gtest
      gtest_main
  )

  target_include_directories(${test_executable} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  if (bullet_version_check_VERSION VERSION_GREATER_EQUAL 2.89)
    target_compile_definitions(${test_executable} PRIVATE
      "BT_BULLET_VERSION_GE_289"
    )
  endif()
  if (bullet_version_check_VERSION VERSION_LESS_EQUAL 3.25)
    target_compile_definitions(${test_executable} PRIVATE
      "BT_BULLET_VERSION_LE_325"
    )
  endif()
  if (bullet_version_check_VERSION VERSION_LESS_EQUAL 3.06)
    target_compile_definitions(${test_executable} PRIVATE
      "BT_BULLET_VERSION_LE_306"
    )
  endif()
  if (bullet_version_check_VERSION VERSION_LESS_EQUAL 3.07)
    target_compile_definitions(${test_executable} PRIVATE
      "BT_BULLET_VERSION_LE_307"
    )
  endif()

  if (DART_HAS_CONTACT_SURFACE_HEADER)
    target_compile_definitions(${test_executable} PRIVATE DART_HAS_CONTACT_SURFACE)
  endif()

  install(TARGETS ${test_executable} DESTINATION ${TEST_INSTALL_DIR})

  configure_common_test("bullet" ${test_executable})
  configure_common_test("bullet-featherstone" ${test_executable})
  configure_common_test("dartsim" ${test_executable})
  configure_common_test("tpe" ${test_executable})
endforeach()
