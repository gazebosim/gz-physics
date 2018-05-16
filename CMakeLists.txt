cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-physics0 VERSION 0.1.0)

#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake1 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
ign_configure_project()


#============================================================================
# Set project-specific options
#============================================================================

#---------------------------------------
# Build all possible plugins by default.
# Users may override this by changing these variables.
option(BUILD_DOUBLE_PENDULUM "Build the double pendulum example" TRUE)


#============================================================================
# Search for project-specific dependencies
#============================================================================
message(STATUS "\n\n-- ====== Finding Dependencies ======")

#--------------------------------------
# Find ignition-math
ign_find_package(ignition-math5 REQUIRED)
set(IGN_MATH_VER ${ignition-math5_VERSION_MAJOR})

#--------------------------------------
# Find ignition-common
ign_find_package(ignition-common2 REQUIRED)
set(IGN_COMMON_VER ${ignition-common2_VERSION_MAJOR})

message(STATUS "-------------------------------------------\n")


#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS)

if(BUILD_DOUBLE_PENDULUM)
  add_subdirectory(double-pendulum)
endif()


#============================================================================
# Create package information
#============================================================================
ign_create_packages()
