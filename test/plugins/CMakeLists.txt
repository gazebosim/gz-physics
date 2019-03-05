if (Bullet_FOUND AND DART_FOUND)
  if(APPLE)
    # workaround for dart issue 1200 until a fix is released
    # https://github.com/dartsim/dart/issues/1200
    # https://github.com/dartsim/dart/pull/1209
    link_directories(/usr/local/opt/console_bridge/lib)
  endif()
  add_library(MockDoublePendulum SHARED
    BulletDoublePendulum.cc
    DARTDoublePendulum.cc)
  target_link_libraries(MockDoublePendulum PRIVATE ignition-common${IGN_COMMON_VER}::core)
endif()
add_library(MockEntities SHARED MockEntities.cc)
add_library(MockFrames SHARED frames.cc)
add_library(MockJoints SHARED MockJoints.cc)

list(APPEND plugins
    MockEntities
    MockFrames
    MockJoints)
if (TARGET MockDoublePendulum)
  list(APPEND plugins MockDoublePendulum)
endif()

foreach(plugin IN LISTS plugins)
  target_link_libraries(${plugin}
    PRIVATE
      ${PROJECT_LIBRARY_TARGET_NAME})
endforeach()

if (TARGET MockDoublePendulum)
  target_include_directories(MockDoublePendulum PUBLIC ${BULLET_INCLUDE_DIRS})
  target_link_libraries(MockDoublePendulum
      PUBLIC
        ${BULLET_LIBRARIES}
        ${DART_LIBRARIES})
  target_compile_definitions(MockDoublePendulum PRIVATE
    "IGNITION_PHYSICS_RESOURCE_DIR=\"${IGNITION_PHYSICS_RESOURCE_DIR}\"")
endif()
