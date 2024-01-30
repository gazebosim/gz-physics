include(GzBenchmark)

set(tests
  ExpectData.cc
)

gz_add_benchmarks(SOURCES ${tests} LIB_DEPS gz-physics-test)
