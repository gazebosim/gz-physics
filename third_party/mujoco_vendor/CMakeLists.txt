# Using FetchContent causes dependencies of mujoco to leak into with gz-physics.
include(ExternalProject)
set(MUJOCO_INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/install")

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

# Create the IMPORTED target for gz-physics to link against
add_library(mujoco_internal INTERFACE)
add_dependencies(mujoco_internal mujoco_ext)

# Create the missing include directory so CMake doesn't complain at configure time
file(MAKE_DIRECTORY "${MUJOCO_INSTALL_DIR}/include")

target_include_directories(mujoco_internal INTERFACE "${MUJOCO_INSTALL_DIR}/include")

if(WIN32)
    # Windows links against the .lib, but needs the .dll at runtime
    target_link_libraries(mujoco_internal INTERFACE "${MUJOCO_IMPLIB_FILE}")
    install(PROGRAMS "${MUJOCO_SHARED_FILE}" DESTINATION bin)
    install(FILES "${MUJOCO_IMPLIB_FILE}" DESTINATION lib)
else()
    # Unix links directly against the .so / .dylib
    target_link_libraries(mujoco_internal INTERFACE "${MUJOCO_SHARED_FILE}")
    install(DIRECTORY "${MUJOCO_INSTALL_DIR}/lib/"
            DESTINATION lib
            FILES_MATCHING 
            PATTERN "*mujoco.so*"
            PATTERN "*mujoco*.dylib*"
    )
endif()

# Create the nice namespace alias for the main project to use. This is also
# what the target would be if mujoco was built/installed externally.
add_library(mujoco::mujoco ALIAS mujoco_internal)
