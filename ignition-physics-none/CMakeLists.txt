
ign_get_libsources_and_unittests(sources gtest_sources)

get_filename_component(lib_name ${CMAKE_CURRENT_LIST_DIR} NAME)

add_library(${lib_name} ${sources})

target_link_libraries(${lib_name}
  ignition-physics0
  ${IGNITION-COMMON_LIBRARIES}
)

target_include_directories(${lib_name} PRIVATE
  ${IGNITION-COMMON_INCLUDE_DIRS}
)

# This allows us to access utils/test_config.h
include_directories(${PROJECT_BINARY_DIR})

ign_build_tests(
  TYPE PLUGIN
  SOURCES ${gtest_sources})
