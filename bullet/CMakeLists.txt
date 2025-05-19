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
    gz-common::gz-common
    gz-math::eigen3)

# Note that plugins are currently being installed in 2 places: /lib and the engine-plugins dir
install(TARGETS ${bullet_plugin} DESTINATION ${GZ_PHYSICS_ENGINE_RELATIVE_INSTALL_DIR})

if (WIN32)
  # disable MSVC inherit via dominance warning
  target_compile_options(${bullet_plugin} PUBLIC "/wd4250")
endif()
