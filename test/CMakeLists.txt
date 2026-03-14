#============================================================================
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
