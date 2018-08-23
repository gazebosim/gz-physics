add_library(MockEntities SHARED MockEntities.cc)
add_library(MockFrames SHARED frames.cc)

foreach(plugin MockEntities MockFrames)
  target_link_libraries(${plugin}
    PRIVATE
      ${PROJECT_LIBRARY_TARGET_NAME})
endforeach()
