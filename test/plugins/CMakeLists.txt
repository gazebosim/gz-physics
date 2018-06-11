add_library(MockPhysics SHARED mock.cc)

target_link_libraries(MockPhysics
  PRIVATE
    ${PROJECT_LIBRARY_TARGET_NAME}
    Eigen3::Eigen)
target_compile_options(MockPhysics PRIVATE -fkeep-inline-functions)
