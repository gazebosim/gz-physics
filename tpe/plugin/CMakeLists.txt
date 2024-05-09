# This component expresses custom features of the tpe plugin, which can
# expose native tpe data types.
gz_add_component(tpe INTERFACE
  DEPENDS_ON_COMPONENTS sdf mesh
  GET_TARGET_NAME features)

target_link_libraries(${features} INTERFACE ${PROJECT_LIBRARY_TARGET_NAME}-tpelib)
target_include_directories(${features} SYSTEM INTERFACE)

gz_get_libsources_and_unittests(sources test_sources)

# TODO(MXG): Think about a gz_add_plugin(~) macro for gz-cmake
set(engine_name tpe-plugin)
gz_add_component(${engine_name}
  SOURCES ${sources}
  DEPENDS_ON_COMPONENTS tpe
  GET_TARGET_NAME tpe_plugin)

set(tpelib_dir ${PROJECT_SOURCE_DIR}/tpe)
target_include_directories(${tpe_plugin} PRIVATE ${tpelib_dir})

target_link_libraries(${tpe_plugin}
  PUBLIC
    ${features}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-math${GZ_MATH_VER}::eigen3
  PRIVATE
    # We need to link this, even when the profiler isn't used to get headers.
    gz-common${GZ_COMMON_VER}::profiler
)

# Note that plugins are currently being installed in 2 places: /lib and the engine-plugins dir
install(TARGETS ${tpe_plugin} DESTINATION ${GZ_PHYSICS_ENGINE_RELATIVE_INSTALL_DIR})

# The library created by `gz_add_component` includes the gz-physics version
# (i.e. libgz-physics1-name-plugin.so), but for portability,
# we also install an unversioned symlink into the same versioned folder.
set(versioned ${CMAKE_SHARED_LIBRARY_PREFIX}${tpe_plugin}${CMAKE_SHARED_LIBRARY_SUFFIX})
set(unversioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_NO_VERSION_LOWER}-${engine_name}${CMAKE_SHARED_LIBRARY_SUFFIX})
if (WIN32)
  # disable MSVC inherit via dominance warning
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4250")
  INSTALL(CODE "EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy
      ${GZ_PHYSICS_ENGINE_INSTALL_DIR}\/${versioned}
      ${GZ_PHYSICS_ENGINE_INSTALL_DIR}\/${unversioned})")
else()
  EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${versioned} ${unversioned})
  INSTALL(FILES ${PROJECT_BINARY_DIR}/${unversioned} DESTINATION ${GZ_PHYSICS_ENGINE_RELATIVE_INSTALL_DIR})
endif()

gz_build_tests(
  TYPE UNIT_tpe
  SOURCES ${test_sources}
  LIB_DEPS
    ${features}
    gz-physics-test
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
  TEST_LIST tests
  ENVIRONMENT
    GZ_PHYSICS_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  INCLUDE_DIRS
    ${tpelib_dir})

foreach(test ${tests})
  target_compile_definitions(${test} PRIVATE
    "tpe_plugin_LIB=\"$<TARGET_FILE:${tpe_plugin}>\"")
endforeach()
