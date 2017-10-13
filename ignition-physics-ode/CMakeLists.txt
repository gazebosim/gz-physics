
file(GLOB source_files "*.cc")
list(SORT source_files)

file(GLOB gtest_source_files "*_TEST.cc")
list(SORT gtest_source_files)

set(sources "")
set(gtest_sources "")

# Remove tests from source files and collect them into the list of tests
foreach(test_file ${gtest_source_files})

  list(REMOVE_ITEM source_files ${test_file})

  get_filename_component(test ${test_file} NAME)
  list(APPEND gtest_sources ${test})

endforeach(test_file)

# Collect the source file names
foreach(source_file ${source_files})

  get_filename_component(source ${source_file} NAME)
  list(APPEND sources ${source})

endforeach(source_file)

get_filename_component(lib_name ${CMAKE_CURRENT_LIST_DIR} NAME)
ign_add_library(${lib_name} ${sources})

target_link_libraries(${lib_name}
  ignition-physics
  ${ODE_LIBRARY}
  pthread
  ${IGNITION-COMMON_LIBRARIES})

target_include_directories(${lib_name} PRIVATE
  ${ODE_INCLUDE_DIR}
  ${IGNITION-COMMON_INCLUDE_DIRS})

set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")

ign_install_library(${lib_name} ${lib_name})

# This allows us to access utils/test_config.h
include_directories(${PROJECT_BINARY_DIR})

set(TEST_TYPE "PLUGIN")
ign_build_tests(${gtest_sources})