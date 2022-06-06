set(TEST_TYPE "INTEGRATIONNEW")

set(tests
  EntityManagementFeatures
)

link_directories(${PROJECT_BINARY_DIR}/test)

set(PHYSICS_COMPONENTS bullet dartsim tpe)

if (UNIX AND NOT APPLE)
  configure_file(all_symbols_have_version.bash.in ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash @ONLY)
  add_test(NAME NEWINTEGRATION_versioned_symbols
    COMMAND bash ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash $<TARGET_FILE:${PROJECT_LIBRARY_TARGET_NAME}>)

  foreach(comp ${PHYSICS_COMPONENTS})
   set(comp_target_name "${PROJECT_LIBRARY_TARGET_NAME}-${comp}-plugin")
   add_test(NAME "NEWINTEGRATION_versioned_symbols_${comp}"
    COMMAND bash ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash $<TARGET_FILE:${comp_target_name}>)
  endforeach()
endif()

ign_build_tests(
    TYPE
      NEWINTEGRATION
    SOURCES
      ${tests}
    LIB_DEPS
      ignition-plugin${IGN_PLUGIN_VER}::loader
      ignition-common5::ignition-common5
)
