include(IgnBenchmark)

set(tests
  ExpectData.cc
)

gz_add_benchmarks(SOURCES ${tests})
