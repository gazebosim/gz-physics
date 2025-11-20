gz_get_libsources_and_unittests(sources test_sources)

gz_add_component(mujoco-plugin
  DEPENDS_ON_COMPONENTS sdf mesh
  SOURCES ${sources}
  GET_TARGET_NAME plugin_target)

target_link_libraries(${plugin_target}
  PUBLIC
    mujoco::mujoco
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-heightmap
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    gz-common::gz-common
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
    ${mujoco_plugin}
    gz-physics-test
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-heightmap
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
  TEST_LIST tests
  ENVIRONMENT
    GZ_PHYSICS_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

foreach(test ${tests})
  target_compile_definitions(${test} PRIVATE
    "mujoco_plugin_LIB=\"$<TARGET_FILE:${plugin_target}>\"")

  # Helps when we want to build a single test after making changes to mujoco_plugin
  add_dependencies(${test} ${mujoco_plugin})
endforeach()
