if(NOT gz-common${GZ_COMMON_VER}-geospatial_FOUND)
  # Skip adding this component
  return()
endif()
gz_add_component(heightmap INTERFACE
  GET_TARGET_NAME heightmap)

target_link_libraries(${heightmap}
  INTERFACE
    gz-common${GZ_COMMON_VER}::geospatial)

install(
  DIRECTORY include/
  DESTINATION "${GZ_INCLUDE_INSTALL_DIR_FULL}")
