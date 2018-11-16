add_library(MockDoublePendulum SHARED DARTDoublePendulum.cc)
add_library(MockEntities SHARED MockEntities.cc)
add_library(MockFrames SHARED frames.cc)
add_library(MockJoints SHARED MockJoints.cc)

foreach(plugin
    MockDoublePendulum
    MockEntities
    MockFrames
    MockJoints)
  target_link_libraries(${plugin}
    PRIVATE
      ${PROJECT_LIBRARY_TARGET_NAME})
endforeach()

target_link_libraries(MockDoublePendulum INTERFACE ${DART_LIBRARIES})
target_include_directories(MockDoublePendulum SYSTEM INTERFACE ${DART_INCLUDE_DIRS})
