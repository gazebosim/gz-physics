cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-physics6 VERSION 6.0.0)

#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake3 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
ign_configure_project(
  REPLACE_IGNITION_INCLUDE_PATH gz/physics
  VERSION_SUFFIX pre1)

#============================================================================
# Set project-specific options
#============================================================================

option(ENABLE_PROFILER "Enable Gazebo Profiler" FALSE)

if(ENABLE_PROFILER)
  add_definitions("-DIGN_PROFILER_ENABLE=1")
else()
  add_definitions("-DIGN_PROFILER_ENABLE=0")
endif()

#============================================================================
# Search for project-specific dependencies
#============================================================================
message(STATUS "\n\n-- ====== Finding Dependencies ======")

#--------------------------------------
# Find ignition-common
ign_find_package(ignition-common5
  COMPONENTS geospatial graphics profiler
  REQUIRED_BY heightmap mesh dartsim tpe tpelib bullet)
set(IGN_COMMON_VER ${ignition-common5_VERSION_MAJOR})

#--------------------------------------
# Find ignition-math
ign_find_package(ignition-math7 REQUIRED COMPONENTS eigen3)
set(IGN_MATH_VER ${ignition-math7_VERSION_MAJOR})

#--------------------------------------
# Find ignition-plugin
ign_find_package(ignition-plugin2 REQUIRED COMPONENTS all)
set(IGN_PLUGIN_VER ${ignition-plugin2_VERSION_MAJOR})

#--------------------------------------
# Find ignition-utils
ign_find_package(ignition-utils2 REQUIRED COMPONENTS cli)
set(IGN_UTILS_VER ${ignition-utils2_VERSION_MAJOR})

#--------------------------------------
# Find Eigen
ign_find_package(EIGEN3 REQUIRED)

#--------------------------------------
# Find SDFormat for the SDF features
ign_find_package(sdformat13
  REQUIRED_BY sdf dartsim tpe bullet)

#--------------------------------------
# Find dartsim for the dartsim plugin wrapper
ign_find_package(DART
  COMPONENTS
    collision-bullet
    collision-ode
    utils
    utils-urdf
  EXTRA_ARGS CONFIG
  VERSION 6.9
  REQUIRED_BY dartsim
  PKGCONFIG dart
  PKGCONFIG_VER_COMPARISON >=)

#--------------------------------------
# Find bullet for the bullet plugin wrapper
ign_find_package(IgnBullet
  VERSION 2.87
  REQUIRED_BY bullet
  PKGCONFIG bullet
  PKGCONFIG_VER_COMPARISON >=)

message(STATUS "-------------------------------------------\n")

set(GZ_PHYSICS_RESOURCE_DIR "${CMAKE_SOURCE_DIR}/resources")

# Plugin install dirs
set(GZ_PHYSICS_ENGINE_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/engine-plugins
)

#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS
  COMPONENTS sdf heightmap mesh dartsim tpe bullet)


#============================================================================
# Create package information
#============================================================================
ign_create_packages()


#============================================================================
# Create documentation
#============================================================================
configure_file(${CMAKE_SOURCE_DIR}/api.md.in ${CMAKE_BINARY_DIR}/api.md)
configure_file(${CMAKE_SOURCE_DIR}/tutorials.md.in ${CMAKE_BINARY_DIR}/tutorials.md)

ign_create_docs(
  API_MAINPAGE_MD "${CMAKE_BINARY_DIR}/api.md"
  TUTORIALS_MAINPAGE_MD "${CMAKE_BINARY_DIR}/tutorials.md"
  TAGFILES
    "${IGNITION-COMMON_DOXYGEN_TAGFILE} = ${IGNITION-COMMON_API_URL}"
    "${IGNITION-PLUGIN_DOXYGEN_TAGFILE} = ${IGNITION-PLUGIN_API_URL}"
    "${IGNITION-MATH_DOXYGEN_TAGFILE} = ${IGNITION-MATH_API_URL}"
  )

file(COPY ${CMAKE_SOURCE_DIR}/tutorials/img/ DESTINATION ${CMAKE_BINARY_DIR}/doxygen/html/img/)
