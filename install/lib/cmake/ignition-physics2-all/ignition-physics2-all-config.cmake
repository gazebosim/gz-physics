# - Config to retrieve all components of the ignition-physics2 package
#
# This should only be invoked by ignition-physics2-config.cmake.
#
# To retrieve this meta-package, use:
# find_package(ignition-physics2 COMPONENTS all)
#
# This creates the target ignition-physics2::all which will link to all known
# components of ignition-physics2, including the core library.
#
# This also creates the variable ignition-physics2_ALL_LIBRARIES
#
################################################################################

cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(ignition-physics2_ALL_CONFIG_INCLUDED)
  return()
endif()
set(ignition-physics2_ALL_CONFIG_INCLUDED TRUE)

if(NOT ignition-physics2-all_FIND_QUIETLY)
  message(STATUS "Looking for all libraries of ignition-physics2 -- found version 2.1.0")
endif()


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was ignition-all-config.cmake.in                            ########

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

# Find the core library
find_dependency(ignition-physics2 2.1.0 EXACT)

# Find the component libraries
find_dependency(ignition-physics2-sdf 2.1.0 EXACT)
find_dependency(ignition-physics2-mesh 2.1.0 EXACT)
find_dependency(ignition-physics2-dartsim 2.1.0 EXACT)
find_dependency(ignition-physics2-dartsim-plugin 2.1.0 EXACT)
find_dependency(ignition-physics2-tpelib 2.1.0 EXACT)
find_dependency(ignition-physics2-tpe 2.1.0 EXACT)
find_dependency(ignition-physics2-tpe-plugin 2.1.0 EXACT)

if(NOT TARGET ignition-physics2::ignition-physics2-all)
  include("${CMAKE_CURRENT_LIST_DIR}/ignition-physics2-all-targets.cmake")

  add_library(ignition-physics2::all INTERFACE IMPORTED)
  set_target_properties(ignition-physics2::all PROPERTIES
    INTERFACE_LINK_LIBRARIES "ignition-physics2::ignition-physics2-all")

endif()

# Create the "requested" target if it does not already exist
if(NOT TARGET ignition-physics2::requested)
  add_library(ignition-physics2::requested INTERFACE IMPORTED)
endif()

# Link the "all" target to the "requested" target
get_target_property(ign_requested_components ignition-physics2::requested INTERFACE_LINK_LIBRARIES)
if(NOT ign_requested_components)
  set_target_properties(ignition-physics2::requested PROPERTIES
    INTERFACE_LINK_LIBRARIES "ignition-physics2::ignition-physics2-all")
else()
  set_target_properties(ignition-physics2::requested PROPERTIES
    INTERFACE_LINK_LIBRARIES "${ign_requested_components};ignition-physics2::ignition-physics2-all")
endif()

set(ignition-physics2_ALL_LIBRARIES ignition-physics2::ignition-physics2-all)
