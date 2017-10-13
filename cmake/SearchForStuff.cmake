include (${project_cmake_dir}/Utils.cmake)
include (CheckCXXSourceCompiles)

include (${project_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

include (${project_cmake_dir}/FindODE.cmake)

########################################
# Find ignition math
set(IGNITION-MATH_REQUIRED_MAJOR_VERSION 3)
if (NOT DEFINED IGNITION-MATH_LIBRARY_DIRS AND NOT DEFINED IGNITION-MATH_INCLUDE_DIRS AND NOT DEFINED IGNITION-MATH_LIBRARIES)
  find_package(ignition-math${IGNITION-MATH_REQUIRED_MAJOR_VERSION} QUIET)
  if (NOT ignition-math${IGNITION-MATH_REQUIRED_MAJOR_VERSION}_FOUND)
    message(STATUS "Looking for ignition-math${IGNITION-MATH_REQUIRED_MAJOR_VERSION}-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition math${IGNITION-MATH_REQUIRED_MAJOR_VERSION} library.")
  else()
    message(STATUS "Looking for ignition-math${IGNITION-MATH_REQUIRED_MAJOR_VERSION}-config.cmake - found")
  endif()
endif()

########################################
# Include man pages stuff
include (${project_cmake_dir}/Ronn2Man.cmake)
add_manpage_target()

# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()

########################################
# Find ignition common
set(IGNITION-COMMON_REQUIRED_MAJOR_VERSION 0)
if (NOT DEFINED IGNITION-COMMON_LIBRARY_DIRS AND NOT DEFINED IGNITION-COMMON_INCLUDE_DIRS AND NOT DEFINED IGNITION-COMMON_LIBRARIES)
  find_package(ignition-common${IGNITION-COMMON_REQUIRED_MAJOR_VERSION} QUIET)
  if (NOT ignition-common${IGNITION-COMMON_REQUIRED_MAJOR_VERSION}_FOUND)
    message(STATUS "Looking for ignition-common${IGNITION-COMMON_REQUIRED_MAJOR_VERSION}-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition common${IGNITION-COMMON_REQUIRED_MAJOR_VERSION} library.")
  else()
    message(STATUS "Looking for ignition-common${IGNITION-COMMON_REQUIRED_MAJOR_VERSION}-config.cmake - found")
  endif()
endif()

#################################################
# Find DART
set(DART_MIN_REQUIRED_VERSION 6)
find_package(DART ${DART_MIN_REQUIRED_VERSION} COMPONENTS utils utils-urdf)
if (DART_FOUND)
  message(STATUS "Looking for DART - found")
  set(HAVE_DART TRUE)
else()
  message(STATUS "Looking for DART - not found")
  BUILD_WARNING("DART ${DART_MIN_REQUIRED_VERSION} not found. For "
    "ignition-physics-dart, please install libdart${DART_MIN_REQUIRED_VERSION}-dev.")
  set(HAVE_DART FALSE)
endif(DART_FOUND)
