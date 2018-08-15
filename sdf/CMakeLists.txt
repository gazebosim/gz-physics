
ign_add_component(sdf INTERFACE
  GET_TARGET_NAME sdf)

target_link_libraries(${sdf} INTERFACE ${SDFormat_LIBRARIES})
target_include_directories(${sdf} INTERFACE ${SDFormat_INCLUDE_DIRS})

install(
  DIRECTORY include/
  DESTINATION "${IGN_INCLUDE_INSTALL_DIR_FULL}")
