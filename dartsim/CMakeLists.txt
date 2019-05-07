
# This component expresses custom features of the dartsim plugin, which can
# expose native dartsim data types.
ign_add_component(dartsim INTERFACE
  DEPENDS_ON_COMPONENTS sdf mesh
  GET_TARGET_NAME features)

target_link_libraries(${features} INTERFACE ${DART_LIBRARIES})
target_include_directories(${features} SYSTEM INTERFACE ${DART_INCLUDE_DIRS})

message("DART_LIBRARIES:${DART_LIBRARIES}")

install(
  DIRECTORY include/
  DESTINATION "${IGN_INCLUDE_INSTALL_DIR_FULL}")

ign_get_libsources_and_unittests(sources test_sources)

# TODO(MXG): Think about an ign_add_plugin(~) macro for ign-cmake
ign_add_component(dartsim-plugin
  SOURCES ${sources}
  DEPENDS_ON_COMPONENTS dartsim
  GET_TARGET_NAME dartsim_plugin)

set(dependencies
  ${features}
  ${PROJECT_LIBRARY_TARGET_NAME}-sdf
  ${PROJECT_LIBRARY_TARGET_NAME}-mesh
  ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  ignition-math${IGN_MATH_VER}::eigen3)

file(GLOB precompiled_src "src/precompiled/*.cc")
add_library(dartsim-precompiled-types SHARED src/precompiled/Link.cc)
target_link_libraries(dartsim-precompiled-types PUBLIC ${dependencies})
target_compile_options(dartsim-precompiled-types PUBLIC "-ftime-report")
target_compile_definitions(dartsim-precompiled-types PRIVATE ignition_physics2_dartsim_precompile_EXPORTS)

target_link_libraries(${dartsim_plugin} PUBLIC ${dependencies})

ign_build_tests(
  TYPE UNIT
  SOURCES ${test_sources}
  LIB_DEPS
    ${features}
    ignition-plugin${IGN_PLUGIN_VER}::loader
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    dartsim-precompiled-types
  TEST_LIST tests)

foreach(test ${tests})

  target_compile_definitions(${test} PRIVATE
    "dartsim_plugin_LIB=\"$<TARGET_FILE:${dartsim_plugin}>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/worlds/\""
    "IGNITION_PHYSICS_RESOURCE_DIR=\"${IGNITION_PHYSICS_RESOURCE_DIR}\"")

endforeach()

if(TARGET UNIT_FindFeatures_TEST)

  target_compile_definitions(UNIT_FindFeatures_TEST PRIVATE
    "dartsim_plugin_LIB=\"$<TARGET_FILE:${dartsim_plugin}>\"")

endif()
