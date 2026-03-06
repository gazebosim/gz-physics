cmake_minimum_required(VERSION 3.22.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(gz-physics VERSION 9.0.0)

#============================================================================
# Find gz-cmake
#============================================================================
find_package(gz-cmake REQUIRED)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

gz_configure_project(VERSION_SUFFIX pre1)

#============================================================================
# Set project-specific options
#============================================================================

option(ENABLE_PROFILER "Enable Gazebo Profiler" FALSE)

if(ENABLE_PROFILER)
  add_definitions("-DGZ_PROFILER_ENABLE=1")
else()
  add_definitions("-DGZ_PROFILER_ENABLE=0")
endif()

#============================================================================
# Search for project-specific dependencies
#============================================================================
message(STATUS "\n\n-- ====== Finding Dependencies ======")

#--------------------------------------
# Use Fetchcontent for Mujoco during development since a binary package is not
# available yet.
# TODO(azeey) Remove this and depend on a system package before release
#
include(FetchContent)
FetchContent_Declare(
  mujoco
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco
  GIT_TAG 3.5.0
)

set(MUJOCO_BUILD_EXAMPLES OFF)
set(MUJOCO_BUILD_TESTS OFF)
set(MUJOCO_BUILD_SIMULATE OFF)
set(MUJOCO_TEST_PYTHON_UTIL OFF)
FetchContent_MakeAvailable(mujoco)

# Ensure LTO is disabled for the mujoco target and its dependencies so it
# builds a standard shared library
if(TARGET mujoco)
  set_target_properties(mujoco PROPERTIES INTERPROCEDURAL_OPTIMIZATION OFF)

  # Workaround for https://github.com/google-deepmind/mujoco/pull/3069
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(mujoco PRIVATE -Wno-error=stringop-truncation)
  endif()

  # MuJoCo's build system forces CMAKE_INTERPROCEDURAL_OPTIMIZATION ON for its
  # dependencies (ccd, lodepng, etc.). We need to disable it for them too,
  # otherwise they are built as LTO bitcode which cannot be properly bundled
  # into a non-LTO shared library using --whole-archive.
  # See, issue: https://github.com/google-deepmind/mujoco/issues/2904
  set(MUJOCO_DEPS ccd lodepng qhullstatic_r tinyobjloader tinyxml2)
  foreach(dep ${MUJOCO_DEPS})
    if(TARGET ${dep})
      set_target_properties(${dep} PROPERTIES INTERPROCEDURAL_OPTIMIZATION OFF)
    endif()
  endforeach()

  if(NOT APPLE AND NOT MSVC)
    # Prevent bundled static libraries (like tinyxml2) from exporting symbols,
    # avoiding conflicts with system versions.
    target_link_options(mujoco PRIVATE "-Wl,--exclude-libs,ALL")
  endif()
endif()

# We use target_link_libraries to use the proper mujoco::mujoco alias
add_library(mujoco_fetched INTERFACE IMPORTED)
target_link_libraries(mujoco_fetched INTERFACE mujoco::mujoco)
target_link_options(mujoco_fetched INTERFACE
  "-Wl,-rpath,$<TARGET_FILE_DIR:mujoco>")

#--------------------------------------
# Find gz-common
gz_find_package(gz-common
  COMPONENTS geospatial graphics profiler
  REQUIRED_BY heightmap mesh dartsim tpe tpelib bullet)

# This is only used for test support
gz_find_package(gz-common REQUIRED COMPONENTS testing)

#--------------------------------------
# Find gz-math
gz_find_package(gz-math REQUIRED COMPONENTS eigen3)

#--------------------------------------
# Find gz-plugin
gz_find_package(gz-plugin REQUIRED COMPONENTS all)

#--------------------------------------
# Find gz-utils
gz_find_package(gz-utils REQUIRED COMPONENTS cli)

#--------------------------------------
# Find Eigen
gz_find_package(EIGEN3 REQUIRED)

#--------------------------------------
# Find SDFormat for the SDF features
gz_find_package(sdformat
  REQUIRED_BY sdf dartsim tpe bullet)

#--------------------------------------
# Find dartsim for the dartsim plugin wrapper
gz_find_package(DART
  COMPONENTS
    collision-bullet
    collision-ode
    utils
    utils-urdf
  CONFIG
  VERSION 6.10
  REQUIRED_BY dartsim
  PKGCONFIG dart
  PKGCONFIG_VER_COMPARISON >=)

#--------------------------------------
# Find bullet for the bullet plugin wrapper
gz_find_package(GzBullet
  VERSION 2.87
  REQUIRED_BY bullet bullet-featherstone
  PKGCONFIG bullet
  PKGCONFIG_VER_COMPARISON >=)

#--------------------------------------
# Find mujoco for the mujoco plugin wrapper
# gz_find_package(mujoco
#   REQUIRED_BY mujoco
#   PKGCONFIG_IGNORE # TODO(azeey) Revisit this
#   CONFIG)

message(STATUS "-------------------------------------------\n")

# Plugin install dirs
set(GZ_PHYSICS_ENGINE_RELATIVE_INSTALL_DIR
  ${GZ_LIB_INSTALL_DIR}/gz-${GZ_DESIGNATION}-${PROJECT_VERSION_MAJOR}/engine-plugins
)
set(GZ_PHYSICS_ENGINE_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${GZ_PHYSICS_ENGINE_RELATIVE_INSTALL_DIR}
)

#============================================================================
# Configure the build
#============================================================================
gz_configure_build(QUIT_IF_BUILD_ERRORS
  COMPONENTS sdf heightmap mesh dartsim tpe bullet bullet-featherstone mujoco)


#============================================================================
# Create package information
#============================================================================
gz_create_packages()


#============================================================================
# Create documentation
#============================================================================
configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)

gz_create_docs(
  API_MAINPAGE_MD "${CMAKE_BINARY_DIR}/api.md"
  TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md"
  TAGFILES
    "${GZ-COMMON_DOXYGEN_TAGFILE} = ${GZ-COMMON_API_URL}"
    "${GZ-PLUGIN_DOXYGEN_TAGFILE} = ${GZ-PLUGIN_API_URL}"
    "${GZ-MATH_DOXYGEN_TAGFILE} = ${GZ-MATH_API_URL}"
  )

file(COPY ${CMAKE_SOURCE_DIR}/tutorials/img/ DESTINATION ${CMAKE_BINARY_DIR}/doxygen/html/img/)
