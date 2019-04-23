include(IgnBenchmark OPTIONAL RESULT_VARIABLE IgnBenchmark_FOUND)

if (IgnBenchmark_FOUND)
  set(tests
    ExpectData.cc
  )

  ign_add_benchmarks(SOURCES ${benchmark_list})
endif()
