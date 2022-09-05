# This component expresses custom features of the bullet plugin, which can
# expose native bullet data types.
gz_add_component(bullet INTERFACE
  DEPENDS_ON_COMPONENTS sdf mesh
  GET_TARGET_NAME features)

link_directories(${BULLET_LIBRARY_DIRS})
target_link_libraries(${features} INTERFACE GzBullet::GzBullet)

gz_get_libsources_and_unittests(sources test_sources)

# TODO(MXG): Think about a gz_add_plugin(~) macro for gz-cmake
set(engine_name bullet-plugin)
gz_add_component(${engine_name}
  SOURCES ${sources}
  DEPENDS_ON_COMPONENTS bullet
  GET_TARGET_NAME bullet_plugin)

target_link_libraries(${bullet_plugin}
  PUBLIC
    ${features}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-math${GZ_MATH_VER}::eigen3)

# Note that plugins are currently being installed in 2 places: /lib and the engine-plugins dir
install(TARGETS ${bullet_plugin} DESTINATION ${GZ_PHYSICS_ENGINE_INSTALL_DIR})

# Install redirection headers
install(
  DIRECTORY include/
  DESTINATION "${GZ_INCLUDE_INSTALL_DIR_FULL}")

# The library created by `gz_add_component` includes the gz-physics version
# (i.e. libgz-physics1-name-plugin.so), but for portability,
# we also install an unversioned symlink into the same versioned folder.
set(versioned ${CMAKE_SHARED_LIBRARY_PREFIX}${bullet_plugin}${CMAKE_SHARED_LIBRARY_SUFFIX})
set(unversioned ${CMAKE_SHARED_LIBRARY_PREFIX}${PROJECT_NAME_NO_VERSION_LOWER}-${engine_name}${CMAKE_SHARED_LIBRARY_SUFFIX})
if (WIN32)
  # disable MSVC inherit via dominance warning
  target_compile_options(${bullet_plugin} PUBLIC "/wd4250")
  INSTALL(CODE "EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E copy
      ${GZ_PHYSICS_ENGINE_INSTALL_DIR}\/${versioned}
      ${GZ_PHYSICS_ENGINE_INSTALL_DIR}\/${unversioned})")
else()
  EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${versioned} ${unversioned})
  INSTALL(FILES ${PROJECT_BINARY_DIR}/${unversioned} DESTINATION ${GZ_PHYSICS_ENGINE_INSTALL_DIR})
endif()
EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${versioned} ${unversioned})
INSTALL(FILES ${PROJECT_BINARY_DIR}/${unversioned} DESTINATION ${GZ_PHYSICS_ENGINE_INSTALL_DIR})
