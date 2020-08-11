#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ignition-physics2::ignition-physics2-tpe-plugin" for configuration "RelWithDebInfo"
set_property(TARGET ignition-physics2::ignition-physics2-tpe-plugin APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(ignition-physics2::ignition-physics2-tpe-plugin PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libignition-physics2-tpe-plugin.so.2.1.0"
  IMPORTED_SONAME_RELWITHDEBINFO "libignition-physics2-tpe-plugin.so.2"
  )

list(APPEND _IMPORT_CHECK_TARGETS ignition-physics2::ignition-physics2-tpe-plugin )
list(APPEND _IMPORT_CHECK_FILES_FOR_ignition-physics2::ignition-physics2-tpe-plugin "${_IMPORT_PREFIX}/lib/libignition-physics2-tpe-plugin.so.2.1.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
