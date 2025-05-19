# This component expresses custom features of the dartsim plugin, which can
# expose native dartsim data types.
gz_add_component(dartsim INTERFACE
  DEPENDS_ON_COMPONENTS sdf heightmap mesh
  GET_TARGET_NAME features)

target_link_libraries(${features} INTERFACE ${DART_LIBRARIES})
target_include_directories(${features} SYSTEM INTERFACE ${DART_INCLUDE_DIRS})
if (MSVC)
  # needed by DART, see https://github.com/dartsim/dart/issues/753
  target_compile_options(${features} INTERFACE "/permissive-")
endif()

gz_get_libsources_and_unittests(sources test_sources)

# TODO(MXG): Think about a gz_add_plugin(~) macro for gz-cmake
set(engine_name dartsim-plugin)
gz_add_component(${engine_name}
  SOURCES ${sources}
  DEPENDS_ON_COMPONENTS dartsim
  GET_TARGET_NAME dartsim_plugin)

target_link_libraries(${dartsim_plugin}
  PUBLIC
    ${features}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-heightmap
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    gz-common::gz-common
    gz-common::geospatial
    gz-math::eigen3
  PRIVATE
    # We need to link this, even when the profiler isn't used to get headers.
    gz-common::profiler
)

# The Gazebo fork of DART contains additional code that allows customizing
# contact constraints. We check for the presence of "ContactSurface.hpp", which
# was added to enable these customizations, to detect if the feature is
# available.
include(CheckIncludeFileCXX)
if (MSVC)
  set(CMAKE_REQUIRED_FLAGS "/std:c++14")
else()
  set(CMAKE_REQUIRED_FLAGS "-std=c++14")
endif()
set(CMAKE_REQUIRED_INCLUDES "${DART_INCLUDE_DIRS};${EIGEN3_INCLUDE_DIRS}")
set(CMAKE_REQUIRED_QUIET false)

CHECK_INCLUDE_FILE_CXX(dart/constraint/ContactSurface.hpp DART_HAS_CONTACT_SURFACE_HEADER)
if (DART_HAS_CONTACT_SURFACE_HEADER)
  target_compile_definitions(${dartsim_plugin} PRIVATE DART_HAS_CONTACT_SURFACE)
else()
  message(STATUS "The version of DART does not support Contact constraint customizations.")
endif()

# Note that plugins are currently being installed in 2 places: /lib and the engine-plugins dir
install(TARGETS ${dartsim_plugin} DESTINATION ${GZ_PHYSICS_ENGINE_RELATIVE_INSTALL_DIR})

if (WIN32)
  # disable MSVC inherit via dominance warning
  target_compile_options(${dartsim_plugin} PUBLIC "/wd4250")
endif()

# Testing
gz_build_tests(
  TYPE UNIT
  SOURCES ${test_sources}
  LIB_DEPS
    ${features}
    ${dartsim_plugin}
    gz-physics-test
    gz-common::geospatial
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
    "dartsim_plugin_LIB=\"$<TARGET_FILE:${dartsim_plugin}>\"")

  if (DART_HAS_CONTACT_SURFACE_HEADER)
    target_compile_definitions(${test} PRIVATE DART_HAS_CONTACT_SURFACE)
  endif()

  # Helps when we want to build a single test after making changes to dartsim_plugin
  add_dependencies(${test} ${dartsim_plugin})
endforeach()
