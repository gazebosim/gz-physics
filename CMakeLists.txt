cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)


#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake0 REQUIRED)

#============================================================================
# Set up the project
#============================================================================
ign_configure_project(physics 0.1.0)

# Configure documentation uploader
configure_file("tools/upload_doc.sh.in" ${CMAKE_BINARY_DIR}/upload_doc.sh @ONLY)

#============================================================================
# Set project-specific options
#============================================================================

# No project-specific options yet


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


#============================================================================
# Create package information
#============================================================================
ign_create_packages()
