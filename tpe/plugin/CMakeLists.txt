# This component expresses custom features of the tpe plugin, which can
# expose native tpe data types.
ign_add_component(tpe INTERFACE
  DEPENDS_ON_COMPONENTS sdf mesh
  GET_TARGET_NAME features)

target_link_libraries(${features} INTERFACE ${PROJECT_LIBRARY_TARGET_NAME}-tpe)
target_include_directories(${features} SYSTEM INTERFACE)

ign_get_libsources_and_unittests(sources test_sources)

# TODO(MXG): Think about an ign_add_plugin(~) macro for ign-cmake
ign_add_component(tpe-plugin
  SOURCES ${sources}
  DEPENDS_ON_COMPONENTS tpe
  GET_TARGET_NAME tpe_plugin)

target_link_libraries(${tpe_plugin}
  PUBLIC
    ${features}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-math${IGN_MATH_VER}::eigen3)

# We need to link this, even when the profiler isn't used to get headers.
target_link_libraries(${tpe_plugin}
  PRIVATE
  ignition-common${IGN_COMMON_VER}::profiler
)

# Note that plugins are currently being installed in 2 places: /lib and the engine-plugins dir
install(TARGETS ${tpe_plugin} DESTINATION ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR})

# The library created by `ign_add_component` includes the ign-physics version
# (i.e. libignition-physics1-name-plugin.so), but for portability,
# we also install an unversioned symlink into the same versioned folder.
set(versioned ${CMAKE_SHARED_LIBRARY_PREFIX}${tpe_plugin}${CMAKE_SHARED_LIBRARY_SUFFIX})
set(unversioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_NO_VERSION_LOWER}-tpe-plugin${CMAKE_SHARED_LIBRARY_SUFFIX})
if (WIN32)
  # create_symlink requires cmake 3.13 on windows
  cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
endif()
EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${versioned} ${unversioned})
INSTALL(FILES ${PROJECT_BINARY_DIR}/${unversioned} DESTINATION ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR})

ign_build_tests(
  TYPE UNIT_tpe
  SOURCES ${test_sources}
  LIB_DEPS
    ${features}
    ignition-plugin${IGN_PLUGIN_VER}::loader
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
  TEST_LIST tests)

foreach(test ${tests})

  target_compile_definitions(${test} PRIVATE
    "tpe_plugin_LIB=\"$<TARGET_FILE:${tpe_plugin}>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/worlds/\""
    "IGNITION_PHYSICS_RESOURCE_DIR=\"${IGNITION_PHYSICS_RESOURCE_DIR}\"")

endforeach()


