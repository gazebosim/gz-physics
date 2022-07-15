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
)

add_subdirectory(gtest_vendor)
add_subdirectory(benchmark)
add_subdirectory(common_test)
add_subdirectory(plugins)
add_subdirectory(helpers)
add_subdirectory(integration)
add_subdirectory(performance)
add_subdirectory(regression)
add_subdirectory(static_assert)
