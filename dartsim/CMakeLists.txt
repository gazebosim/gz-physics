# This component expresses custom features of the dartsim plugin, which can
# expose native dartsim data types.
ign_add_component(dartsim INTERFACE
  DEPENDS_ON_COMPONENTS sdf mesh
  GET_TARGET_NAME features)

target_link_libraries(${features} INTERFACE ${DART_LIBRARIES})
target_include_directories(${features} SYSTEM INTERFACE ${DART_INCLUDE_DIRS})

install(
  DIRECTORY include/
  DESTINATION "${IGN_INCLUDE_INSTALL_DIR_FULL}")

ign_get_libsources_and_unittests(sources test_sources)

# TODO(MXG): Think about an ign_add_plugin(~) macro for ign-cmake
set(engine_name dartsim-plugin)
ign_add_component(${engine_name}
  SOURCES ${sources}
  DEPENDS_ON_COMPONENTS dartsim
  GET_TARGET_NAME dartsim_plugin)

target_link_libraries(${dartsim_plugin}
  PUBLIC
    ${features}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-math${IGN_MATH_VER}::eigen3
  PRIVATE
    # We need to link this, even when the profiler isn't used to get headers.
    ignition-common${IGN_COMMON_VER}::profiler
)

# The ignition fork of DART contains additional code that allows customizing 
# contact constraints. We check for the presence of "ContactSurface.hpp", which 
# was added to enable these customizations, to detect if the feature is 
# available.
include(CheckIncludeFileCXX)
set(CMAKE_REQUIRED_INCLUDES "${DART_INCLUDE_DIRS};${EIGEN3_INCLUDE_DIRS}")
set(CMAKE_REQUIRED_QUIET false)

CHECK_INCLUDE_FILE_CXX(dart/constraint/ContactSurface.hpp DART_HAS_CONTACT_SURFACE_HEADER)
if (DART_HAS_CONTACT_SURFACE_HEADER)
  target_compile_definitions(${dartsim_plugin} PRIVATE DART_HAS_CONTACT_SURFACE)
else()
  message(STATUS "The version of DART does not support Contact constraint customizations.")
endif()

# Note that plugins are currently being installed in 2 places: /lib and the engine-plugins dir
install(TARGETS ${dartsim_plugin} DESTINATION ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR})

# The library created by `ign_add_component` includes the ign-physics version
# (i.e. libignition-physics1-name-plugin.so), but for portability,
# we also install an unversioned symlink into the same versioned folder.
set(versioned ${CMAKE_SHARED_LIBRARY_PREFIX}${dartsim_plugin}${CMAKE_SHARED_LIBRARY_SUFFIX})
set(unversioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_NO_VERSION_LOWER}-${engine_name}${CMAKE_SHARED_LIBRARY_SUFFIX})
if (WIN32)
  # create_symlink requires cmake 3.13 on windows
  cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
endif()
EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${versioned} ${unversioned})
INSTALL(FILES ${PROJECT_BINARY_DIR}/${unversioned} DESTINATION ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR})

# Testing
ign_build_tests(
  TYPE UNIT
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
    "dartsim_plugin_LIB=\"$<TARGET_FILE:${dartsim_plugin}>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/worlds/\""
    "IGNITION_PHYSICS_RESOURCE_DIR=\"${IGNITION_PHYSICS_RESOURCE_DIR}\"")

  if (DART_HAS_CONTACT_SURFACE_HEADER)
    target_compile_definitions(${test} PRIVATE DART_HAS_CONTACT_SURFACE)
  endif()

  # Helps when we want to build a single test after making changes to dartsim_plugin
  add_dependencies(${test} ${dartsim_plugin})
endforeach()

foreach(test UNIT_FindFeatures_TEST UNIT_RequestFeatures_TEST)
  if(TARGET ${test})
    target_compile_definitions(${test} PRIVATE
      "dartsim_plugin_LIB=\"$<TARGET_FILE:${dartsim_plugin}>\"")
  endif()
endforeach()
