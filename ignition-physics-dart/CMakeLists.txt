
# TODO: When the Components branch of ign-cmake is ready, use that instead of
# manually creating this component.

set(DART_VER 6)
find_package(DART ${DART_VER} COMPONENTS utils utils-urdf)
if(DART_FOUND)
  message(STATUS "Looking for DART - found")
else()
  message(STATUS "Looking for DART - not found")
  message(STATUS "DART ${DART_VER} is required for ignition-physics-dart, "
    "but could not be found. We will not build ignition-physics-dart.")
  return()
endif()

ign_get_libsources_and_unittests(sources gtest_sources)

get_filename_component(lib_name ${CMAKE_CURRENT_LIST_DIR} NAME)
add_library(${lib_name} ${sources})
target_link_libraries(${lib_name}
  ignition-physics0
  ${IGNITION-COMMON_LIBRARIES}
  ${DART_LIBRARIES}
  ${DART-UTILS_LIBRARIES}
  ${DART-UTILS-URDF_LIBRARIES})

target_include_directories(${lib_name} PRIVATE
  ignition-physics
  ${DART_INCLUDE_DIRS}
  ${IGNITION-COMMON_INCLUDE_DIRS})

# This allows us to access utils/test_config.h
include_directories(${PROJECT_BINARY_DIR})

ign_build_tests(
  TYPE PLUGIN
  SOURCES ${gtest_sources})
