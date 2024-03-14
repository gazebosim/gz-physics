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
  ENVIRONMENT
    GZ_PHYSICS_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  LIB_DEPS
    gz-physics-test
)
