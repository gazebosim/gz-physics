set(TEST_TYPE "BENCHMARK")

find_package(benchmark)
if (NOT benchmark_FOUND)
  message(WARNING "Unable to find google benchmark (libbenchmark-dev). Disabling benchmarks.")
else()
  set(tests
    ExpectData.cc
  )

  ign_build_executables(
    PREFIX "${TEST_TYPE}_"
    SOURCES ${tests}
    LIB_DEPS benchmark::benchmark
    EXEC_LIST benchmark_list
  )

  include(IgnBenchmark)
  ign_add_run_benchmarks_target(TARGETS ${benchmark_list})
endif()
