gz_get_libsources_and_unittests(sources test_sources)

gz_add_component(mujoco-plugin
  DEPENDS_ON_COMPONENTS sdf mesh
  SOURCES ${sources}
  GET_TARGET_NAME plugin_target)

target_link_libraries(${plugin_target}
  PUBLIC
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-heightmap
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    gz-common::gz-common
  PRIVATE
    mujoco::mujoco
)

# Note that plugins are currently being installed in 2 places:
# /lib and the engine-plugins dir
install(TARGETS ${plugin_target}
        DESTINATION ${GZ_PHYSICS_ENGINE_RELATIVE_INSTALL_DIR})

# Testing
gz_build_tests(
  TYPE UNIT_mujoco
  SOURCES ${test_sources}
  LIB_DEPS
    ${plugin_target}
    gz-physics-test
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-heightmap
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    mujoco::mujoco
  TEST_LIST tests
  ENVIRONMENT
    GZ_PHYSICS_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

foreach(test ${tests})
  target_compile_definitions(${test} PRIVATE
    "mujoco_plugin_LIB=\"$<TARGET_FILE:${plugin_target}>\"")

  # Helps when we want to build a single test after making changes to plugin_target
  add_dependencies(${test} ${plugin_target})
endforeach()

# Integration tests drive the plugin through the public gz-physics API rather
# than reaching into plugin internals. They live in test/ rather than src/
# because gz_get_libsources_and_unittests classifies everything matching
# src/*_TEST.cc as a unit test.
gz_build_tests(
  TYPE INTEGRATION_mujoco
  SOURCES
    test/ThreadPool.cc
  LIB_DEPS
    ${plugin_target}
    gz-physics-test
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    Eigen3::Eigen
    mujoco::mujoco
  TEST_LIST integration_tests
  ENVIRONMENT
    GZ_PHYSICS_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

foreach(test ${integration_tests})
  target_compile_definitions(${test} PRIVATE
    "mujoco_plugin_LIB=\"$<TARGET_FILE:${plugin_target}>\"")

  add_dependencies(${test} ${plugin_target})
endforeach()
