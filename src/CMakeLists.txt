gz_get_libsources_and_unittests(sources gtest_sources)
gz_create_core_library(SOURCES ${sources} CXX_STANDARD 17)
gz_add_get_install_prefix_impl(GET_INSTALL_PREFIX_FUNCTION gz::physics::getInstallPrefix
                               GET_INSTALL_PREFIX_HEADER gz/physics/InstallationDirectories.hh
                               OVERRIDE_INSTALL_PREFIX_ENV_VARIABLE GZ_PHYSICS_INSTALL_PREFIX)

set_property(
  SOURCE InstallationDirectories.cc
  PROPERTY COMPILE_DEFINITIONS
  GZ_PHYSICS_ENGINE_RELATIVE_INSTALL_DIR="${GZ_PHYSICS_ENGINE_RELATIVE_INSTALL_DIR}"
)

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    gz-math::gz-math
    gz-utils::gz-utils
    gz-plugin::register
    Eigen3::Eigen)

if(WIN32)
  target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
    PRIVATE shlwapi)
endif()

gz_build_tests(
  TYPE UNIT
  SOURCES ${gtest_sources}
  LIB_DEPS
      gz-physics-test
  ENVIRONMENT
    GZ_PHYSICS_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
)

if(TARGET UNIT_FindFeatures_TEST)
  target_link_libraries(UNIT_FindFeatures_TEST
    gz-plugin::loader)
endif()
