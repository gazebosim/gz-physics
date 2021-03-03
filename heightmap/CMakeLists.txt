ign_add_component(heightmap INTERFACE
  GET_TARGET_NAME heightmap)

target_link_libraries(${heightmap}
  INTERFACE
    ignition-common${IGN_COMMON_VER}::graphics)

install(
  DIRECTORY include/
  DESTINATION "${IGN_INCLUDE_INSTALL_DIR_FULL}")
