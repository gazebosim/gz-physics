
# This component expresses custom features of the dartsim plugin, which can
# expose native dartsim data types.
ign_add_component(dartsim INTERFACE
  DEPENDS_ON_COMPONENTS sdf mesh
  GET_TARGET_NAME features)

target_link_libraries(${features} INTERFACE ${DART_LIBRARIES})

if(USE_VENDOR_DART)
  target_include_directories(${features} INTERFACE
    $<BUILD_INTERFACE:${DART_VENDOR_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${BULLET_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
    $<INSTALL_INTERFACE:${IGN_INCLUDE_INSTALL_DIR_FULL}/vendor/dart>)
else()
  target_include_directories(${features} SYSTEM INTERFACE ${DART_INCLUDE_DIRS})
endif()

if (MSVC)
  # needed by DART, see https://github.com/dartsim/dart/issues/753
  target_compile_options(${features} INTERFACE "/permissive-")
endif()

install(
  DIRECTORY include/
  DESTINATION "${IGN_INCLUDE_INSTALL_DIR_FULL}")

ign_get_libsources_and_unittests(sources test_sources)

# TODO(MXG): Think about an ign_add_plugin(~) macro for ign-cmake
set(engine_name dartsim-plugin)
ign_add_component(${engine_name}
  SOURCES ${sources}
  DEPENDS_ON_COMPONENTS dartsim
  GET_TARGET_NAME dartsim_plugin)

add_dependencies(${PROJECT_LIBRARY_TARGET_NAME}-dartsim-plugin dart)
if(USE_VENDOR_DART)
  add_dependencies(${PROJECT_LIBRARY_TARGET_NAME}-dartsim-plugin dart_project)
endif()

target_link_libraries(${dartsim_plugin}
  PUBLIC
    ${features}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-math${IGN_MATH_VER}::eigen3
  PRIVATE
    # We need to link this, even when the profiler isn't used to get headers.
    ignition-common${IGN_COMMON_VER}::profiler
)

# Note that plugins are currently being installed in 2 places: /lib and the engine-plugins dir
install(TARGETS ${dartsim_plugin} DESTINATION ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR})

if(USE_VENDOR_DART)
  # Set RPATH/RUNPATH to point to engine install dir. This makes possible
  # to find embedded dart libraries withotu interfere with oter system directories
  set_target_properties(${dartsim_plugin} PROPERTIES
      INSTALL_RPATH ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR})

  foreach(lib ${DART_VENDOR_LIBRARY_PATH} ${DART_VENDOR_LIBRARY_LINK_PATH}
	      ${DART_COLLISION_VENDOR_LIBRARY_PATH} ${DART_COLLISION_VENDOR_LIBRARY_LINK_PATH}
	      ${DART_ODEL_VENDOR_LIBRARY_PATH} ${DART_ODEL_VENDOR_LIBRARY_LINK_PATH})
	  #      install(FILES ${lib}
	  # DESTINATION ${CMAKE_INSTALL_PREFIX}/${IGN_LIB_INSTALL_DIR})
      install(FILES ${lib}
	DESTINATION ${IGNITION_PHYSICS_ENGINE_INSTALL_DIR})
  endforeach()
endif()

# The library created by `ign_add_component` includes the ign-physics version
# (i.e. libignition-physics1-name-plugin.so), but for portability,
# we also install an unversioned symlink into the same versioned folder.
set(versioned ${CMAKE_SHARED_LIBRARY_PREFIX}${dartsim_plugin}${CMAKE_SHARED_LIBRARY_SUFFIX})
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

# Testing
ign_build_tests(
  TYPE UNIT
  SOURCES ${test_sources}
  LIB_DEPS
    ${features}
    ignition-plugin${IGN_PLUGIN_VER}::loader
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
  TEST_LIST tests)

foreach(test ${tests})
   if(USE_VENDOR_DART)
     set_tests_properties(${test} PROPERTIES
	 ENVIRONMENT "LD_LIBRARY_PATH=${DART_VENDOR_LIBRARY_DIR}")
  endif()

  target_compile_definitions(${test} PRIVATE
    "dartsim_plugin_LIB=\"$<TARGET_FILE:${dartsim_plugin}>\""
    "TEST_WORLD_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/worlds/\""
    "IGNITION_PHYSICS_RESOURCE_DIR=\"${IGNITION_PHYSICS_RESOURCE_DIR}\"")

  # Helps when we want to build a single test after making changes to dartsim_plugin
  add_dependencies(${test} ${dartsim_plugin})
endforeach()

foreach(test UNIT_FindFeatures_TEST UNIT_RequestFeatures_TEST)
  if(TARGET ${test})
    target_compile_definitions(${test} PRIVATE
      "dartsim_plugin_LIB=\"$<TARGET_FILE:${dartsim_plugin}>\"")
  endif()
endforeach()
