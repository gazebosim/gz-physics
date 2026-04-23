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
# Tests are split into two gz_build_tests calls because of a dynamic linker
# conflict: tests that load the plugin via gz-plugin must NOT also link
# mujoco::mujoco directly. When a test binary links libmujoco.so directly AND
# loads the mujoco plugin (which itself links libmujoco.so), the dynamic linker
# ends up with two instances of libmujoco in the same process, causing a
# segfault during MuJoCo's internal state initialization.
#
# - Plugin-loading tests: use only ${plugin_target}, no direct mujoco linkage.
# - Unit tests that include Base.hh directly (e.g. WorldInfoDestructor_TEST):
#   need mujoco::mujoco for headers and symbols, but do not load the plugin.
list(FILTER test_sources EXCLUDE REGEX "WorldInfoDestructor_TEST\\.cc$")
gz_build_tests(
  TYPE UNIT_mujoco
  SOURCES ${test_sources}
  LIB_DEPS
    ${plugin_target}
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

  # Helps when we want to build a single test after making changes to plugin_target
  add_dependencies(${test} ${plugin_target})
endforeach()

gz_build_tests(
  TYPE UNIT_mujoco
  SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/WorldInfoDestructor_TEST.cc
  LIB_DEPS
    ${plugin_target}
    gz-physics-test
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-heightmap
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    mujoco::mujoco
  TEST_LIST destructor_tests
  ENVIRONMENT
    GZ_PHYSICS_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

foreach(test ${destructor_tests})
  add_dependencies(${test} ${plugin_target})
endforeach()
