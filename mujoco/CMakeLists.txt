gz_get_libsources_and_unittests(sources test_sources)

gz_add_component(mujoco-plugin
  DEPENDS_ON_COMPONENTS sdf mesh
  SOURCES ${sources}
  GET_TARGET_NAME plugin_target)

target_link_libraries(${plugin_target}
  PUBLIC
    ${features}
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${PROJECT_LIBRARY_TARGET_NAME}-heightmap
    ${PROJECT_LIBRARY_TARGET_NAME}-mesh
    gz-common::gz-common
    gz-math::eigen3
)

# Note that plugins are currently being installed in 2 places:
# /lib and the engine-plugins dir
install(TARGETS ${plugin_target}
        DESTINATION ${GZ_PHYSICS_ENGINE_RELATIVE_INSTALL_DIR})

endforeach()
