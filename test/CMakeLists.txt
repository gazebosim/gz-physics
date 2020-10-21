if (NOT BUILD_TESTING)
  return()
endif()

include_directories (
  ${PROJECT_SOURCE_DIR}/test/gtest/include
  ${PROJECT_SOURCE_DIR}/test/gtest
  ${PROJECT_SOURCE_DIR}/test
  ${CMAKE_BINARY_DIR}/include
)

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
set_property(TARGET gtest_main PROPERTY CXX_STANDARD ${c++standard})
set_property(TARGET gtest PROPERTY CXX_STANDARD ${c++standard})
set(GTEST_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest.a")
set(GTEST_MAIN_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest_main.a")

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
# This gives us access to src/utils headers
include_directories(${PROJECT_SOURCE_DIR}/src ${GTEST_INCLUDE_DIRS})

#============================================================================
# Do a fake install of ign-math in order to test the examples.
#============================================================================
set(FAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/fake/install")

file(MAKE_DIRECTORY ${FAKE_INSTALL_PREFIX})

include(ExternalProject)
ExternalProject_Add(
  FAKE_INSTALL

  SOURCE_DIR "${CMAKE_SOURCE_DIR}"
  EXCLUDE_FROM_ALL 1
  LOG_CONFIGURE 1
  LOG_BUILD 1
  LOG_INSTALL 1
  CMAKE_ARGS
    "-DBUILD_TESTING=OFF"
    "-DCMAKE_INSTALL_PREFIX=${FAKE_INSTALL_PREFIX}"
)

add_subdirectory(benchmark)
add_subdirectory(plugins)
add_subdirectory(integration)
add_subdirectory(performance)
add_subdirectory(regression)
add_subdirectory(static_assert)
