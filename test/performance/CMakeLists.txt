gz_get_sources(tests)

# ExpectData test causes lcov to hang
# see gz-cmake issue 25
if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "COVERAGE")
  list(REMOVE_ITEM tests
    ExpectData.cc)
endif()

gz_build_tests(
  TYPE PERFORMANCE
  SOURCES ${tests}
  INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/test
  ENVIRONMENT
    GZ_PHYSICS_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/test
)
