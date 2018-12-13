if (Bullet_FOUND AND DART_FOUND)
  add_library(MockDoublePendulum SHARED
    BulletDoublePendulum.cc
    DARTDoublePendulum.cc)
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
  target_link_libraries(MockDoublePendulum PUBLIC ${console_bridge_LIBRARIES})
  target_compile_definitions(MockDoublePendulum PRIVATE
    "IGNITION_PHYSICS_RESOURCE_DIR=\"${IGNITION_PHYSICS_RESOURCE_DIR}\"")
endif()
