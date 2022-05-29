if (DART_FOUND)
  add_library(MockDoublePendulum SHARED DARTDoublePendulum.cc)
  target_link_libraries(MockDoublePendulum PRIVATE ignition-common${IGN_COMMON_VER}::core)
endif()
add_library(MockEntities SHARED MockEntities.cc)
add_library(MockFrames SHARED frames.cc)
add_library(MockJoints SHARED MockJoints.cc)

list(APPEND plugins
    MockEntities
    MockFrames
    MockJoints)
if (DART_FOUND)
  list(APPEND plugins MockDoublePendulum)
endif()

foreach(plugin IN LISTS plugins)
  target_link_libraries(${plugin}
    PRIVATE
      ${PROJECT_LIBRARY_TARGET_NAME})
endforeach()

if (DART_FOUND)
  target_link_libraries(MockDoublePendulum PUBLIC ${DART_LIBRARIES})
  target_compile_definitions(MockDoublePendulum PRIVATE
    "GZ_PHYSICS_RESOURCE_DIR=\"${GZ_PHYSICS_RESOURCE_DIR}\"")
  if (MSVC)
    # needed by DART, see https://github.com/dartsim/dart/issues/753
    target_compile_options(MockDoublePendulum PUBLIC "/permissive-")
    # disable MSVC inherit via dominance warning
    target_compile_options(MockDoublePendulum PUBLIC "/wd4250")
  endif()
endif()
