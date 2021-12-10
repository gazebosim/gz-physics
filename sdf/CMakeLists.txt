
ign_add_component(sdf INTERFACE
  GET_TARGET_NAME sdf)

target_link_libraries(${sdf} INTERFACE sdformat${SDFORMAT_VER}::sdformat${SDFORMAT_VER})

install(
  DIRECTORY include/
  DESTINATION "${IGN_INCLUDE_INSTALL_DIR_FULL}")
