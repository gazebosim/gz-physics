set(TEST_TYPE "COMMON_TEST")

set(tests
  basic_test
)

link_directories(${PROJECT_BINARY_DIR}/test)

set(PHYSICS_COMPONENTS bullet dartsim tpe)

ign_build_tests(
    TYPE
      COMMON_TEST
    SOURCES
      ${tests}
    LIB_DEPS
      ignition-plugin${IGN_PLUGIN_VER}::loader
      ignition-common5::ignition-common5
)
