# test_symbols.cmake
# This script verifies that the built MuJoCo shared library meets our symbol visibility requirements:
# 1. It must NOT export tinyxml2 symbols (to avoid clashes with system tinyxml2).
# 2. It MUST export required mj_* APIs used by the gz-physics plugin.
#
# Required variables:
# LIB_PATH - absolute path to the MuJoCo shared library file (.so)

message(STATUS "Checking symbols in ${LIB_PATH}")

# Find the 'nm' tool, which is standard on Unix systems for listing symbols.
find_program(NM_PATH nm)

if(NOT NM_PATH)
  message(FATAL_ERROR "nm not found! This tool is required for symbol verification.")
endif()

message(STATUS "Using nm: ${NM_PATH}")

# Run 'nm -g' to list only global (external) symbols.
# If tinyxml2 was correctly hidden by the version script, its symbols should not appear here.
execute_process(
    COMMAND ${NM_PATH} -g ${LIB_PATH}
    OUTPUT_VARIABLE NM_OUTPUT
    RESULT_VARIABLE NM_RESULT
)
if(NOT NM_RESULT EQUAL 0)
  message(FATAL_ERROR "Failed to run nm on ${LIB_PATH}")
endif()

# Convert output to lowercase for case-insensitive matching (mangled symbols might vary).
string(TOLOWER "${NM_OUTPUT}" NM_OUTPUT_LOWER)

# Check for tinyxml2 symbols. They should NOT be present in the global symbol list.
if(NM_OUTPUT_LOWER MATCHES "tinyxml2")
  message(FATAL_ERROR "Found exported tinyxml2 symbols in ${LIB_PATH}. The version script may not have worked correctly.")
endif()

# Check for required mj_* symbols to ensure they are kept intact and exported.
set(REQUIRED_SYMBOLS
    "mj_step"
    "mj_compile"
    "mj_makeData"
)

foreach(SYMBOL ${REQUIRED_SYMBOLS})
  if(NOT NM_OUTPUT MATCHES "${SYMBOL}")
    message(FATAL_ERROR "Missing required symbol ${SYMBOL} in ${LIB_PATH}. It may have been accidentally hidden.")
  endif()
endforeach()

message(STATUS "All symbol checks passed for ${LIB_PATH}")
