include(GzBenchmark)

set(tests
  ExpectData.cc
  CompositeData.cc
)

gz_add_benchmarks(SOURCES ${tests} LIB_DEPS gz-physics-test)
