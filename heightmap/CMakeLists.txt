gz_add_component(heightmap INTERFACE
  GET_TARGET_NAME heightmap)

target_link_libraries(${heightmap}
  INTERFACE
    gz-common::geospatial
    gz-math::eigen3)

install(
  DIRECTORY include/
  DESTINATION "${GZ_INCLUDE_INSTALL_DIR_FULL}")

gz_build_tests(
  TYPE UNIT
  SOURCES src/SampleHeightmap_TEST.cc
  LIB_DEPS ${heightmap})
