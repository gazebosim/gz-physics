gz_add_component(sdf INTERFACE
  GET_TARGET_NAME sdf)

target_link_libraries(${sdf} INTERFACE ${SDFormat_LIBRARIES})

install(
  DIRECTORY include/
  DESTINATION "${IGN_INCLUDE_INSTALL_DIR_FULL}")
