# Using FetchContent causes dependencies of mujoco to leak into with gz-physics.
include(CMakePackageConfigHelpers)
include(ExternalProject)
set(MUJOCO_INSTALL_DIR "${GZ_PHYSICS_MUJOCO_BUILD_DIR}")
set(MUJOCO_VENDOR_PACKAGE_NAME "gz-physics-mujoco_vendor")
set(MUJOCO_VENDOR_CMAKE_INSTALL_DIR
  "${GZ_LIB_INSTALL_DIR}/cmake/${MUJOCO_VENDOR_PACKAGE_NAME}")

if(WIN32)
  set(MUJOCO_IMPLIB_FILE "${MUJOCO_INSTALL_DIR}/lib/mujoco.lib")
  set(MUJOCO_SHARED_FILE "${MUJOCO_INSTALL_DIR}/bin/mujoco.dll")
    set(MUJOCO_BYPRODUCTS "${MUJOCO_SHARED_FILE}" "${MUJOCO_IMPLIB_FILE}")
elseif(APPLE)
    set(MUJOCO_SHARED_FILE "${MUJOCO_INSTALL_DIR}/lib/libmujoco.dylib")
    set(MUJOCO_BYPRODUCTS "${MUJOCO_SHARED_FILE}")
else()
    set(MUJOCO_SHARED_FILE "${MUJOCO_INSTALL_DIR}/lib/libmujoco.so")
    set(MUJOCO_BYPRODUCTS "${MUJOCO_SHARED_FILE}")
endif()

ExternalProject_Add(
  mujoco_ext
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco
  GIT_TAG 3.5.0
  INSTALL_DIR    "${MUJOCO_INSTALL_DIR}"
  BUILD_BYPRODUCTS ${MUJOCO_BYPRODUCTS} # Needed for Ninja
  LOG_BUILD ON
  LOG_OUTPUT_ON_FAILURE ON
  CMAKE_ARGS
    -Wno-dev # Suppress any CMake warnings from mujoco
    -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    -DBUILD_SHARED_LIBS=ON
    -DMUJOCO_BUILD_EXAMPLES=OFF
    -DMUJOCO_BUILD_TESTS=OFF
    -DMUJOCO_BUILD_SIMULATE=OFF
    -DMUJOCO_TEST_PYTHON_UTIL=OFF
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
)

add_library(mujoco_vendor INTERFACE)
add_dependencies(mujoco_vendor mujoco_ext)

# Create the missing include directory so CMake doesn't complain at configure time
file(MAKE_DIRECTORY "${GZ_PHYSICS_MUJOCO_BUILD_INCLUDE_DIR}")
file(MAKE_DIRECTORY "${GZ_PHYSICS_MUJOCO_BUILD_LIBRARY_DIR}")

target_include_directories(mujoco_vendor
  INTERFACE
    $<BUILD_INTERFACE:${GZ_PHYSICS_MUJOCO_BUILD_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${GZ_PHYSICS_MUJOCO_INCLUDE_RELATIVE_INSTALL_DIR}>)

target_link_directories(mujoco_vendor
  INTERFACE
    $<BUILD_INTERFACE:${GZ_PHYSICS_MUJOCO_BUILD_LIBRARY_DIR}>
    $<INSTALL_INTERFACE:${GZ_PHYSICS_MUJOCO_LIBRARY_RELATIVE_INSTALL_DIR}>)

target_link_libraries(mujoco_vendor INTERFACE mujoco)

install(TARGETS mujoco_vendor
  EXPORT ${MUJOCO_VENDOR_PACKAGE_NAME}-targets)

install(DIRECTORY "${GZ_PHYSICS_MUJOCO_BUILD_INCLUDE_DIR}/mujoco"
  DESTINATION "${GZ_PHYSICS_MUJOCO_INCLUDE_RELATIVE_INSTALL_DIR}")

if(WIN32)
  install(PROGRAMS "${MUJOCO_SHARED_FILE}"
    DESTINATION "${GZ_PHYSICS_MUJOCO_RUNTIME_RELATIVE_INSTALL_DIR}")
  install(FILES "${MUJOCO_IMPLIB_FILE}"
    DESTINATION "${GZ_PHYSICS_MUJOCO_LIBRARY_RELATIVE_INSTALL_DIR}")
else()
  install(DIRECTORY "${GZ_PHYSICS_MUJOCO_BUILD_LIBRARY_DIR}/"
    DESTINATION "${GZ_PHYSICS_MUJOCO_LIBRARY_RELATIVE_INSTALL_DIR}"
    FILES_MATCHING
    PATTERN "*mujoco.so*"
    PATTERN "*mujoco*.dylib*")
endif()

export(EXPORT ${MUJOCO_VENDOR_PACKAGE_NAME}-targets
  NAMESPACE gz-physics::
  FILE "${PROJECT_BINARY_DIR}/cmake/${MUJOCO_VENDOR_PACKAGE_NAME}-targets.cmake")

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/${MUJOCO_VENDOR_PACKAGE_NAME}-config.cmake.in"
  "${PROJECT_BINARY_DIR}/cmake/${MUJOCO_VENDOR_PACKAGE_NAME}-config.cmake"
  INSTALL_DESTINATION "${MUJOCO_VENDOR_CMAKE_INSTALL_DIR}")

write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/cmake/${MUJOCO_VENDOR_PACKAGE_NAME}-config-version.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion)

install(EXPORT ${MUJOCO_VENDOR_PACKAGE_NAME}-targets
  NAMESPACE gz-physics::
  DESTINATION "${MUJOCO_VENDOR_CMAKE_INSTALL_DIR}"
  FILE "${MUJOCO_VENDOR_PACKAGE_NAME}-targets.cmake")

install(FILES
  "${PROJECT_BINARY_DIR}/cmake/${MUJOCO_VENDOR_PACKAGE_NAME}-config.cmake"
  "${PROJECT_BINARY_DIR}/cmake/${MUJOCO_VENDOR_PACKAGE_NAME}-config-version.cmake"
  DESTINATION "${MUJOCO_VENDOR_CMAKE_INSTALL_DIR}")

# Create the private alias that the plugin uses inside this repository.
add_library(mujoco::mujoco ALIAS mujoco_vendor)
