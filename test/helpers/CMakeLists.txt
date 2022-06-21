add_library(${PROJECT_NAME}_test_lib_loader SHARED
  TestLibLoader.cc
)
target_compile_definitions(${PROJECT_NAME}_test_lib_loader PRIVATE -DTestLibLoader_EXPORTS)
target_link_libraries(${PROJECT_NAME}_test_lib_loader
  PUBLIC
    ${PROJECT_LIBRARY_TARGET_NAME}
    gtest_shared
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-plugin${IGN_PLUGIN_VER}::loader
)
