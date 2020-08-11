# - Config file for the ignition-physics2-sdf component
#
# This should only be invoked by ignition-physics2-config.cmake.
#
# To load this component into your project, use:
# find_package(ignition-physics2 COMPONENTS sdf)
#
# This creates the following targets:
#
#   Component library target                - ignition-physics2::ignition-physics2-sdf
#   Alternative target name                 - ignition-physics2::sdf
#   Core library + all requested components - ignition-physics2::requested
#
# Use target_link_libraries() to link your library or executable to one of the
# above targets.
#
# We also provide the following variable for backwards compatibility, but use of
# this is discouraged:
#
#   ignition-physics2-sdf_LIBRARY  - Component library (actually contains ignition-physics2::sdf)
#
# We will also set ignition-physics2-sdf_FOUND to indicate that the component was found.
#
################################################################################

# We explicitly set the desired cmake version to ensure that the policy settings
# of users or of toolchains do not result in the wrong behavior for our modules.
# Note that the call to find_package(~) will PUSH a new policy stack before
# taking on these version settings, and then that stack will POP after the
# find_package(~) has exited, so this will not affect the cmake policy settings
# of a caller.
cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(ignition-physics2-sdf_CONFIG_INCLUDED)
  return()
endif()
set(ignition-physics2-sdf_CONFIG_INCLUDED TRUE)

if(NOT ignition-physics2-sdf_FIND_QUIETLY)
  message(STATUS "Looking for ignition-physics2-sdf -- found version 2.1.0")
endif()


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was ignition-component-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

# Get access to the find_dependency utility
include(CMakeFindDependencyMacro)

# Find ignition-cmake, because we need its modules in order to find the rest of
# our dependencies.
find_dependency(ignition-cmake2)

# Set the REQUIRED flag for the find_package(~) calls on this component's
# dependencies.
if(ignition-physics2-sdf_FIND_REQUIRED)
  set(ign_package_required REQUIRED)
else()
  set(ign_package_required "")
endif()

# Set the QUIET flag for the find_package(~) calls on this component's
# dependencies.
if(ignition-physics2-sdf_FIND_QUIETLY)
  set(ign_package_quiet QUIET)
else()
  set(ign_package_quiet "")
endif()

# --------------------------------
# Find the dependencies that are specific to this component (if nothing is
# below, then the component has no additional dependencies). We use
# find_package(~) instead of find_dependency(~) here so that we can support
# COMPONENT arguments.
#
# TODO: When we migrate to cmake-3.9+, change these to find_dependency(~),
#       because at that point the find_dependency(~) function will support
#       the COMPONENT argument.
if(NOT ignition-physics2-sdf_FIND_QUIETLY)
  message(STATUS "Searching for dependencies of ignition-physics2-sdf")
endif()
find_package(sdformat9 ${ign_package_quiet} ${ign_package_required})
if(NOT ignition-physics2_CONFIG_INCLUDED)
  find_package(ignition-physics2 2.1.0 EXACT ${ign_package_quiet} ${ign_package_required})
endif()
# --------------------------------

if(NOT TARGET ignition-physics2::ignition-physics2-sdf)
  include("${CMAKE_CURRENT_LIST_DIR}/ignition-physics2-sdf-targets.cmake")

  # Create a simplified imported target name for the sdf library.
  # You can link to this target instead of the sdf library.
  add_library(ignition-physics2::sdf INTERFACE IMPORTED)
  set_target_properties(ignition-physics2::sdf PROPERTIES
    INTERFACE_LINK_LIBRARIES "ignition-physics2::ignition-physics2-sdf")
  # Note: In a future version of cmake, we can replace this with an ALIAS target

endif()

# Create the "requested" target if it does not already exist
if(NOT TARGET ignition-physics2::requested)
  add_library(ignition-physics2::requested INTERFACE IMPORTED)
endif()

# Link the sdf library to the "requested" target
get_target_property(ign_requested_components ignition-physics2::requested INTERFACE_LINK_LIBRARIES)
if(NOT ign_requested_components)
  set_target_properties(ignition-physics2::requested PROPERTIES
    INTERFACE_LINK_LIBRARIES "ignition-physics2::ignition-physics2-sdf")
else()
  set_target_properties(ignition-physics2::requested PROPERTIES
    INTERFACE_LINK_LIBRARIES "${ign_requested_components};ignition-physics2::ignition-physics2-sdf")
endif()

set(ignition-physics2-sdf_LIBRARY ignition-physics2::ignition-physics2-sdf)

# This macro is used by ignition-cmake to automatically configure the pkgconfig
# files for ignition projects.
ign_pkg_config_entry(ignition-physics2-sdf "ignition-physics2-sdf")
