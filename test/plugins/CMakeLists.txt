add_library(MockPhysics SHARED mock.cc)
add_library(MockFrames SHARED frames.cc)

foreach(plugin MockPhysics MockFrames)
  target_link_libraries(${plugin}
    PRIVATE
      ${PROJECT_LIBRARY_TARGET_NAME})
  target_compile_options(${plugin} PRIVATE -fkeep-inline-functions)
endforeach()
