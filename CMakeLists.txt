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


#============================================================================
# Search for project-specific dependencies
#============================================================================
message(STATUS "\n\n-- ====== Finding Dependencies ======")

#--------------------------------------
# Find ignition-math
ign_find_package(ignition-math5 REQUIRED)
set(IGN_MATH_VER ${ignition-math5_VERSION_MAJOR})

#--------------------------------------
# Find ignition-plugin
ign_find_package(ignition-plugin0 REQUIRED COMPONENTS all)
set(IGN_PLUGIN_VER ${ignition-plugin0_VERSION_MAJOR})

#--------------------------------------
# Find Eigen
ign_find_package(EIGEN3 REQUIRED)

message(STATUS "-------------------------------------------\n")


#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS)


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
