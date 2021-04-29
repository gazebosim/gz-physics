# This component expresses custom features of the bullet plugin, which can
# expose native bullet data types.
ign_add_component(bullet INTERFACE
  DEPENDS_ON_COMPONENTS sdf mesh
  GET_TARGET_NAME features)

link_directories(${BULLET_LIBRARY_DIRS})
target_link_libraries(${features} INTERFACE IgnBullet::IgnBullet)

ign_get_libsources_and_unittests(sources test_sources)

# TODO(MXG): Think about an ign_add_plugin(~) macro for ign-cmake
set(engine_name bullet-plugin)
ign_add_component(${engine_name}
  SOURCES ${sources}
  DEPENDS_ON_COMPONENTS bullet
  GET_TARGET_NAME bullet_plugin)

target_link_libraries(${bullet_plugin}
  PUBLIC
    ${features}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-math${IGN_MATH_VER}::eigen3)

# Note that plugins are currently being installed in 2 places: /lib and the engine-plugins dir
install(TARGETS ${bullet_plugin} DESTINATION ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR})

# The library created by `ign_add_component` includes the ign-physics version
# (i.e. libignition-physics1-name-plugin.so), but for portability,
# we also install an unversioned symlink into the same versioned folder.
set(versioned ${CMAKE_SHARED_LIBRARY_PREFIX}${bullet_plugin}${CMAKE_SHARED_LIBRARY_SUFFIX})
set(unversioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_NO_VERSION_LOWER}-${engine_name}${CMAKE_SHARED_LIBRARY_SUFFIX})
if (WIN32)
  # disable MSVC inherit via dominance warning
  target_compile_options(${dartsim_plugin} PUBLIC "/wd4250")
  INSTALL(CODE "EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy
      ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR}\/${versioned}
      ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR}\/${unversioned})")
else()
  EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${versioned} ${unversioned})
  INSTALL(FILES ${PROJECT_BINARY_DIR}/${unversioned} DESTINATION ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR})
endif()
EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${versioned} ${unversioned})
INSTALL(FILES ${PROJECT_BINARY_DIR}/${unversioned} DESTINATION ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR})

# Testing
ign_build_tests(
  TYPE UNIT_bullet
  SOURCES ${test_sources}
  LIB_DEPS
    ${features}
    ignition-plugin${IGN_PLUGIN_VER}::loader
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
  TEST_LIST tests)

foreach(test ${tests})

  target_compile_definitions(${test} PRIVATE
    "bullet_plugin_LIB=\"$<TARGET_FILE:${bullet_plugin}>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/worlds/\""
    "IGNITION_PHYSICS_RESOURCE_DIR=\"${IGNITION_PHYSICS_RESOURCE_DIR}\"")

endforeach()

if(TARGET UNIT_FindFeatures_TEST)

  target_compile_definitions(UNIT_FindFeatures_TEST PRIVATE
    "bullet_plugin_LIB=\"$<TARGET_FILE:${bullet_plugin}>\"")

endif()
