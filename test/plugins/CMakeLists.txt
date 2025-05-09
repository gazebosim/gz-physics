if (DART_FOUND)
  add_library(MockDoublePendulum SHARED DARTDoublePendulum.cc)
  target_link_libraries(MockDoublePendulum PRIVATE gz-common::core)
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
      gz-physics-test
      ${PROJECT_LIBRARY_TARGET_NAME})
endforeach()

if (DART_FOUND)
  # Linking against libdart-collision-bullet causes a segfault on macOS for
  # some reason. We'll remove it since MockDoublePendulum doesn't use bullet
  # for collision checking.
  set(DART_LIBRARIES_NO_BULLET ${DART_LIBRARIES})
  list(REMOVE_ITEM DART_LIBRARIES_NO_BULLET dart-collision-bullet)
  target_link_libraries(MockDoublePendulum PUBLIC ${DART_LIBRARIES_NO_BULLET})
  if (MSVC)
    # needed by DART, see https://github.com/dartsim/dart/issues/753
    target_compile_options(MockDoublePendulum PUBLIC "/permissive-")
    # disable MSVC inherit via dominance warning
    target_compile_options(MockDoublePendulum PUBLIC "/wd4250")
  endif()
endif()
