add_library(MockEntities SHARED MockEntities.cc)
add_library(MockFrames SHARED frames.cc)
add_library(MockJoints SHARED MockJoints.cc)

foreach(plugin
    MockEntities
    MockFrames
    MockJoints)
  target_link_libraries(${plugin}
    PRIVATE
      ${PROJECT_LIBRARY_TARGET_NAME})
endforeach()
