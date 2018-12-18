cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-physics1 VERSION 1.0.0)

#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake2 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
ign_configure_project(VERSION_SUFFIX pre2)


#============================================================================
# Set project-specific options
#============================================================================


#============================================================================
# Search for project-specific dependencies
#============================================================================
message(STATUS "\n\n-- ====== Finding Dependencies ======")

#--------------------------------------
# Find ignition-common
ign_find_package(ignition-common3
  COMPONENTS graphics
  REQUIRED_BY mesh dartsim)
set(IGN_COMMON_VER ${ignition-common3_VERSION_MAJOR})

#--------------------------------------
# Find ignition-math
ign_find_package(ignition-math6 REQUIRED COMPONENTS eigen3)
set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})

#--------------------------------------
# Find ignition-plugin
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS all)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

#--------------------------------------
# Find Eigen
ign_find_package(EIGEN3 REQUIRED)

#--------------------------------------
# Find SDFormat for the SDF features
ign_find_package(sdformat8 REQUIRED_BY sdf dartsim)

#--------------------------------------
# Find dartsim for the dartsim plugin wrapper
ign_find_package(DART
  COMPONENTS collision-bullet
  CONFIG
  VERSION 6
  REQUIRED_BY dartsim
  PKGCONFIG dart
  PKGCONFIG_VER_COMPARISON >=)


message(STATUS "-------------------------------------------\n")

set(IGNITION_PHYSICS_RESOURCE_DIR "${CMAKE_SOURCE_DIR}/resources")

#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS
  COMPONENTS sdf mesh dartsim)


#============================================================================
# Create package information
#============================================================================
ign_create_packages()


#============================================================================
# Create documentation
#============================================================================
ign_create_docs(
  TAGFILES
    "${IGNITION-MATH_DOXYGEN_TAGFILE} = ${IGNITION-MATH_API_URL}"
  )
