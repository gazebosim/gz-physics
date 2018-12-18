
ign_add_component(mesh INTERFACE
  GET_TARGET_NAME mesh)

target_link_libraries(${mesh}
  INTERFACE
    ignition-common${IGN_COMMON_VER}::graphics)

install(
  DIRECTORY include/
  DESTINATION "${IGN_INCLUDE_INSTALL_DIR_FULL}")
