#============================================================================
# Do a fake install of gz-physics in order to test the examples.
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
    "-Dgz-cmake_DIR=${gz-cmake_DIR}"
    "-Dgz-common_DIR=${gz-common_DIR}"
    "-Dgz-common-geospatial_DIR=${gz-common-geospatial_DIR}"
    "-Dgz-common-testing_DIR=${gz-common-testing_DIR}"
    "-Dgz-common-graphics_DIR=${gz-common-graphics_DIR}"
    "-Dgz-common-profiler_DIR=${gz-common-profiler_DIR}"
    "-Dgz-math_DIR=${gz-math_DIR}"
    "-Dgz-math-eigen3_DIR=${gz-math-eigen3_DIR}"
    "-Dgz-plugin_DIR=${gz-plugin_DIR}"
    "-Dgz-plugin-all_DIR=${gz-plugin-all_DIR}"
    "-Dgz-plugin-loader_DIR=${gz-plugin-loader_DIR}"
    "-Dgz-plugin-register_DIR=${gz-plugin-register_DIR}"
    "-Dgz-utils_DIR=${gz-utils_DIR}"
    "-Dgz-utils-log_DIR=${gz-utils-log_DIR}"
    "-Dgz-utils-cli_DIR=${gz-utils-cli_DIR}"
    "-Dsdformat_DIR=${sdformat_DIR}"
)

add_library(gz-physics-test INTERFACE)
target_include_directories(gz-physics-test INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(gz-physics-test INTERFACE
    "TESTING_PROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\""
)
target_link_libraries(gz-physics-test INTERFACE
    gz-plugin::loader
    gz-common::gz-common
    gz-common::testing
)

add_subdirectory(gtest_vendor)
add_subdirectory(benchmark)
add_subdirectory(common_test)
add_subdirectory(plugins)
add_subdirectory(integration)
add_subdirectory(performance)
add_subdirectory(regression)
add_subdirectory(static_assert)
