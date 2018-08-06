
# TODO(MXG): Think about an ign_add_plugin(~) macro for ign-cmake
ign_add_component(dartsim
  SOURCES
    src/plugin.cc
    src/SDFFeatures.cc
  GET_TARGET_NAME dartsim_plugin)

target_link_libraries(${dartsim_plugin}
  PUBLIC
    ${PROJECT_LIBRARY_TARGET_NAME}-sdf
    ${DART_LIBRARIES}
    ignition-math${IGN_MATH_VER}::eigen3)

target_include_directories(${dartsim_plugin}
  SYSTEM PUBLIC ${DART_INCLUDE_DIRS})
