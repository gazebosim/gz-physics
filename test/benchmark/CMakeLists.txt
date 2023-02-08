include(GzBenchmark)

set(tests
  ExpectData.cc
)

gz_add_benchmarks(SOURCES ${tests}
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/test)
