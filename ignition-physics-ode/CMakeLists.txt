# TODO: When the Components branch of ign-cmake is ready, use that instead of
# manually creating this component.

set(ODE_VER 0)
find_package(ODE ${ODE_VER})
if(HAVE_ODE)
  message(STATUS "ODE found")
else()
  message(STATUS "Looking for ODE - not found")
  message(STATUS "ODE is required for ignition-physics-ode, "
    "but could not be found. We will not build ignition-physics-ode.")
  return()
endif()

ign_get_libsources_and_unittests(sources gtest_sources)

get_filename_component(lib_name ${CMAKE_CURRENT_LIST_DIR} NAME)

add_library(${lib_name} ${sources})

if(GCC)
  # This is to work around a known bug in GCC:
  # https://svn.boost.org/trac10/ticket/9240
  # If that bug gets resolved, then this can be removed.
  target_compile_options(${lib_name} PRIVATE -fext-numeric-literals)
endif()

target_link_libraries(${lib_name}
  ignition-physics0
  ${ODE_LIBRARIES}
  pthread
  ${IGNITION-COMMON_LIBRARIES})

target_include_directories(${lib_name} PRIVATE
  ${ODE_INCLUDE_DIR}
  ${IGNITION-COMMON_INCLUDE_DIRS})

# This allows us to access utils/test_config.h
include_directories(${PROJECT_BINARY_DIR})

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")

ign_build_tests(
  TYPE PLUGIN
  SOURCES ${gtest_sources})
