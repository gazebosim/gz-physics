add_library(MockEntities SHARED MockEntities.cc)

target_link_libraries(MockEntities
  PRIVATE
    ${PROJECT_LIBRARY_TARGET_NAME}
    Eigen3::Eigen)
if (NOT MSVC)
  target_compile_options(MockEntities PRIVATE -fkeep-inline-functions)
endif()
