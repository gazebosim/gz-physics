add_library(${PROJECT_NAME}_test_lib_loader SHARED
  TestLibLoader.cc
)
target_compile_definitions(${PROJECT_NAME}_test_lib_loader PRIVATE -DTestLibLoader_EXPORTS)
target_link_libraries(${PROJECT_NAME}_test_lib_loader
  PUBLIC
    ${PROJECT_LIBRARY_TARGET_NAME}
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
)
