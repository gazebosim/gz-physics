include(IgnBenchmark)

set(tests
  ExpectData.cc
)

ign_add_benchmarks(SOURCES ${benchmark_list})
