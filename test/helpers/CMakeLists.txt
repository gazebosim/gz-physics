add_library(${PROJECT_NAME}_test_lib_loader SHARED
  TestLibLoader.cc
  ../gtest/src/gtest-all.cc
)
target_compile_definitions(${PROJECT_NAME}_test_lib_loader PRIVATE -DTestLibLoader_EXPORTS)
target_link_libraries(${PROJECT_NAME}_test_lib_loader
  PUBLIC
    ${PROJECT_LIBRARY_TARGET_NAME}
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
)
