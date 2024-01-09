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

set(GZ_PHYSICS_RESOURCE_DIR "${CMAKE_SOURCE_DIR}/resources")
add_library(gz-physics-test INTERFACE)
target_include_directories(gz-physics-test INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(gz-physics-test INTERFACE
    "GZ_PHYSICS_RESOURCE_DIR=\"${GZ_PHYSICS_RESOURCE_DIR}\""
    "TESTING_PROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\""
)
target_link_libraries(gz-physics-test INTERFACE
      gz-plugin${GZ_PLUGIN_VER}::loader
      gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
      gz-common${GZ_COMMON_VER}::testing
)

add_subdirectory(gtest_vendor)
add_subdirectory(benchmark)
add_subdirectory(common_test)
add_subdirectory(plugins)
add_subdirectory(integration)
add_subdirectory(performance)
add_subdirectory(regression)
add_subdirectory(static_assert)
