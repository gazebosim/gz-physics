cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)


#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake0 REQUIRED)

#============================================================================
# Set up the project
#============================================================================
ign_configure_project(physics 0.1.0)


#============================================================================
# Set project-specific options
#============================================================================

#---------------------------------------
# Build all possible plugins by default.
# Users may override this by changing these variables.
option(BUILD_IGNITION_PHYSICS_DART "Build the DART example physics engine plugin" TRUE)
option(BUILD_IGNITION_PHYSICS_ODE "Build the ODE example physics engine plugin" TRUE)


#============================================================================
# Search for project-specific dependencies
#============================================================================
message(STATUS "\n\n-- ====== Finding Dependencies ======")

#--------------------------------------
# Find ignition-math
set(IGN_MATH_VER 4)
ign_find_package(ignition-math${IGN_MATH_VER} REQUIRED)

#--------------------------------------
# Find ignition-common
set(IGN_COMMON_VER 0)
ign_find_package(ignition-common${IGN_COMMON_VER} REQUIRED)

message(STATUS "-------------------------------------------\n")


#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS)

if(BUILD_IGNITION_PHYSICS_DART)
  add_subdirectory(ignition-physics-dart)
endif()

if(BUILD_IGNITION_PHYSICS_ODE)
  add_subdirectory(ignition-physics-ode)
endif()

#============================================================================
# Create package information
#============================================================================
ign_create_packages()
