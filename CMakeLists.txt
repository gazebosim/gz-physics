cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(ignition-physics3 VERSION 3.3.0)

#============================================================================
# Find ignition-cmake
#============================================================================
find_package(ignition-cmake2 2.8.0 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
ign_configure_project(VERSION_SUFFIX pre1)


#============================================================================
# Set project-specific options
#============================================================================

option(ENABLE_PROFILER "Enable Ignition Profiler" FALSE)

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
ign_find_package(ignition-common3
  COMPONENTS graphics profiler
  REQUIRED_BY mesh dartsim tpe tpelib bullet)
set(IGN_COMMON_VER ${ignition-common3_VERSION_MAJOR})

#--------------------------------------
# Find ignition-math
ign_find_package(ignition-math6 REQUIRED COMPONENTS eigen3)
set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})

#--------------------------------------
# Find ignition-plugin
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS all VERSION 1.1)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

#--------------------------------------
# Find Eigen
ign_find_package(EIGEN3 REQUIRED)

#--------------------------------------
# Find SDFormat for the SDF features
ign_find_package(sdformat10
  REQUIRED_BY sdf dartsim tpe bullet)

#--------------------------------------
# Find dartsim for the dartsim plugin wrapper

# respect user explicit values and also provide a default value for Ubuntu Focal
# to make ground vehicles to work properly
set(DEFAULT_VENDOR_DART OFF)
if(NOT DEFINED USE_VENDOR_DART)
    find_program(LSB_RELEASE_EXEC lsb_release)

    if(LSB_RELEASE_EXEC)
      execute_process(COMMAND ${LSB_RELEASE_EXEC} -rs
        OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
        OUTPUT_STRIP_TRAILING_WHITESPACE)
      if(LSB_RELEASE_ID_SHORT STREQUAL  "20.04")
        message(STATUS "DETECTED Ubuntu 20.04, default USE_VENDOR_DART set to "
                       "ON to make ground vehicles to work properly")
        set(DEFAULT_VENDOR_DART ON)
      endif()
  endif()
endif()

option(USE_VENDOR_DART "Use internal Open Robotics fork of DART" ${DEFAULT_VENDOR_DART})

find_program(LSB_RELEASE_EXEC lsb_release)
if(USE_VENDOR_DART)
    include(ExternalProject)

    set(DART_VENDOR_INSTALL_DIR ${CMAKE_BINARY_DIR}/vendor/dart/install)
    set(DART_VENDOR_INCLUDE_DIRS "${DART_VENDOR_INSTALL_DIR}/include")
    set(DART_VENDOR_LIBRARY_DIR "${DART_VENDOR_INSTALL_DIR}/lib")
    # Three libraries needed
    set(DART_VENDOR_LIBRARY_PATH
	"${DART_VENDOR_LIBRARY_DIR}/libdart.so.6.10.0")
    set(DART_VENDOR_LIBRARY_LINK_PATH
	"${DART_VENDOR_LIBRARY_DIR}/libdart.so.6.10")
    add_library(dart SHARED IMPORTED)
    set_target_properties(dart PROPERTIES
	IMPORTED_LOCATION ${DART_VENDOR_LIBRARY_PATH})

    set(DART_ODEL_VENDOR_LIBRARY_PATH
	"${DART_VENDOR_LIBRARY_DIR}/libdart-external-odelcpsolver.so.6.10.0")
    set(DART_ODEL_VENDOR_LIBRARY_LINK_PATH
	"${DART_VENDOR_LIBRARY_DIR}/libdart-external-odelcpsolver.so.6.10")
    add_library(dart_external_odelcpsolver SHARED IMPORTED)
    set_target_properties(dart_external_odelcpsolver PROPERTIES
	IMPORTED_LOCATION ${DART_ODEL_VENDOR_LIBRARY_PATH})

    set(DART_COLLISION_VENDOR_LIBRARY_PATH
	"${DART_VENDOR_LIBRARY_DIR}/libdart-collision-ode.so.6.10.0")
    set(DART_COLLISION_VENDOR_LIBRARY_LINK_PATH
	"${DART_VENDOR_LIBRARY_DIR}/libdart-collision-ode.so.6.10")
    add_library(dart_collision_ode SHARED IMPORTED)
    set_target_properties(dart_collision_ode PROPERTIES
	IMPORTED_LOCATION ${DART_COLLISION_VENDOR_LIBRARY_PATH})

    set(DART_LIBRARIES dart dart_external_odelcpsolver dart_collision_ode)

    ExternalProject_Add(dart_project
      DOWNLOAD_COMMAND ""
      UPDATE_COMMAND ""
      PREFIX ${CMAKE_BINARY_DIR}/vendor/dart
      INSTALL_DIR ${DART_VENDOR_INSTALL_DIR}
      SOURCE_DIR ${CMAKE_SOURCE_DIR}/vendor/dart
      # Seems a bug since INSTALL_DIR and SOURCE_DIR should handle it
      # https://gitlab.kitware.com/cmake/cmake/-/issues/18790
      CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${DART_VENDOR_INSTALL_DIR}
    )

    add_definitions(-DDART_VERSION=6.10.0)
else()
    ign_find_package(DART
      COMPONENTS
	collision-ode
	utils
	utils-urdf
      EXTRA_ARGS CONFIG
      VERSION 6.9
      REQUIRED_BY dartsim
      PKGCONFIG dart
      PKGCONFIG_VER_COMPARISON >=)
endif()

#--------------------------------------
# Find bullet for the bullet plugin wrapper
ign_find_package(IgnBullet
  VERSION 2.87
  REQUIRED_BY bullet
  PKGCONFIG bullet
  PKGCONFIG_VER_COMPARISON >=)

set(IGNITION_PHYSICS_RESOURCE_DIR "${CMAKE_SOURCE_DIR}/resources")

# Plugin install dirs
set(IGNITION_PHYSICS_ENGINE_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR}/ign-${IGN_DESIGNATION}-${PROJECT_VERSION_MAJOR}/engine-plugins
)

#============================================================================
# Configure the build
#============================================================================
ign_configure_build(QUIT_IF_BUILD_ERRORS
  COMPONENTS sdf mesh dartsim tpe bullet)


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
