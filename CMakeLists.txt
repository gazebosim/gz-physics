cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(gz-physics6 VERSION 6.7.0)

#============================================================================
# Find gz-cmake
#============================================================================
find_package(gz-cmake3 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

gz_configure_project(VERSION_SUFFIX)

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
# Find gz-common
gz_find_package(gz-common5
  COMPONENTS geospatial graphics profiler
  REQUIRED_BY heightmap mesh dartsim tpe tpelib bullet)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

# This is only used for test support
gz_find_package(gz-common5 REQUIRED COMPONENTS testing)

#--------------------------------------
# Find gz-math
gz_find_package(gz-math7 REQUIRED COMPONENTS eigen3)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

#--------------------------------------
# Find gz-plugin
gz_find_package(gz-plugin2 REQUIRED COMPONENTS all)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

#--------------------------------------
# Find gz-utils
gz_find_package(gz-utils2 REQUIRED COMPONENTS cli)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})

#--------------------------------------
# Find Eigen
gz_find_package(EIGEN3 REQUIRED)

#--------------------------------------
# Find SDFormat for the SDF features
gz_find_package(sdformat13
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
  VERSION 6.9
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
  COMPONENTS sdf heightmap mesh dartsim tpe bullet bullet-featherstone)


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
