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

set(_mujoco_test_environment
  GZ_PHYSICS_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

if(WIN32)
  list(APPEND _mujoco_test_environment
    "PATH=${GZ_PHYSICS_MUJOCO_BUILD_RUNTIME_DIR};$ENV{PATH}")
elseif(APPLE)
  set_property(TARGET ${plugin_target} APPEND PROPERTY BUILD_RPATH
    "${GZ_PHYSICS_MUJOCO_BUILD_LIBRARY_DIR}")
  set_property(TARGET ${plugin_target} APPEND PROPERTY INSTALL_RPATH
    "@loader_path/../vendors/mujoco/lib")
elseif(UNIX)
  set_property(TARGET ${plugin_target} APPEND PROPERTY BUILD_RPATH
    "${GZ_PHYSICS_MUJOCO_BUILD_LIBRARY_DIR}")
  set_property(TARGET ${plugin_target} APPEND PROPERTY INSTALL_RPATH
    "$ORIGIN/../vendors/mujoco/lib")
endif()

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
  TEST_LIST tests
  ENVIRONMENT
    ${_mujoco_test_environment}
  INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

foreach(test ${tests})
  target_compile_definitions(${test} PRIVATE
    "mujoco_plugin_LIB=\"$<TARGET_FILE:${plugin_target}>\"")

  # Helps when we want to build a single test after making changes to plugin_target
  add_dependencies(${test} ${plugin_target})
endforeach()
