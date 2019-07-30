# This component expresses custom features of the bullet plugin, which can
# expose native bullet data types.
ign_add_component(bullet INTERFACE
  DEPENDS_ON_COMPONENTS sdf mesh
  GET_TARGET_NAME features)

link_directories(${BULLET_LIBRARY_DIRS})
target_link_libraries(${features} INTERFACE ${BULLET_LIBRARIES})
target_include_directories(${features} SYSTEM INTERFACE ${BULLET_INCLUDE_DIRS})
add_definitions(${BULLET_CFLAGS})

ign_get_libsources_and_unittests(sources test_sources)

# TODO(MXG): Think about an ign_add_plugin(~) macro for ign-cmake
ign_add_component(bullet-plugin
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

# TODO(Jay): build tests