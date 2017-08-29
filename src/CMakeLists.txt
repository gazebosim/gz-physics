include (${project_cmake_dir}/Utils.cmake)

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

message(STATUS "Sources: ${sources}")
message(STATUS "Tests: ${gtest_sources}")
message(STATUS "PLTN: ${PROJECT_LIBRARY_TARGET_NAME}")

ign_add_library(${PROJECT_LIBRARY_TARGET_NAME} ${sources})
target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  ${IGNITION-COMMON_LIBRARIES}
  ${IGNITION-MATH_LIBRARIES})

if(NOT MSVC)
  target_compile_options(${PROJECT_LIBRARY_TARGET_NAME} PUBLIC "-std=c++11")
endif()

ign_install_library(${PROJECT_LIBRARY_TARGET_NAME} ${PROJECT_EXPORT_NAME})
ign_build_tests(${gtest_sources})
