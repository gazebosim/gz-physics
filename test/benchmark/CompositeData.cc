#include <benchmark/benchmark.h>
#include <gz/physics/CompositeData.hh>

struct SomeData1 { int value = 1; };
struct SomeData2 { int value = 1; };
struct SomeData3 { int value = 1; };
struct SomeData4 { int value = 1; };
struct SomeData5 { int value = 1; };
struct SomeData6 { int value = 1; };
struct SomeData7 { int value = 1; };
struct SomeData8 { int value = 1; };
struct SomeData9 { int value = 1; };
struct SomeData10 { int value = 1; };
struct SomeData11 { int value = 1; };
struct SomeData12 { int value = 1; };
struct SomeData13 { int value = 1; };
struct SomeData14 { int value = 1; };
struct SomeData15 { int value = 1; };
struct SomeData16 { int value = 1; };
struct SomeData17 { int value = 1; };
struct SomeData18 { int value = 1; };
struct SomeData19 { int value = 1; };
struct SomeData20 { int value = 1; };

// Helper to populate CompositeData
void PopulateCompositeData(gz::physics::CompositeData& cd, int size)
{
  if (size >= 1) cd.Get<SomeData1>();
  if (size >= 2) cd.Get<SomeData2>();
  if (size >= 3) cd.Get<SomeData3>();
  if (size >= 4) cd.Get<SomeData4>();
  if (size >= 5) cd.Get<SomeData5>();
  if (size >= 6) cd.Get<SomeData6>();
  if (size >= 7) cd.Get<SomeData7>();
  if (size >= 8) cd.Get<SomeData8>();
  if (size >= 9) cd.Get<SomeData9>();
  if (size >= 10) cd.Get<SomeData10>();
  if (size >= 11) cd.Get<SomeData11>();
  if (size >= 12) cd.Get<SomeData12>();
  if (size >= 13) cd.Get<SomeData13>();
  if (size >= 14) cd.Get<SomeData14>();
  if (size >= 15) cd.Get<SomeData15>();
  if (size >= 16) cd.Get<SomeData16>();
  if (size >= 17) cd.Get<SomeData17>();
  if (size >= 18) cd.Get<SomeData18>();
  if (size >= 19) cd.Get<SomeData19>();
  if (size >= 20) cd.Get<SomeData20>();
}

static void BM_CompositeData_Construct_Destruct(benchmark::State& state) {
  gz::physics::CompositeData cd;

  for (auto _ : state) {
    PopulateCompositeData(cd, state.range(0));
  }
}

static void BM_CompositeData_Get_Existing(benchmark::State& state) {
  gz::physics::CompositeData cd;
  PopulateCompositeData(cd, state.range(0));

  for (auto _ : state) {
    benchmark::DoNotOptimize(cd.Get<SomeData1>());
  }
}

static void BM_CompositeData_Query_Existing(benchmark::State& state) {
  gz::physics::CompositeData cd;
  PopulateCompositeData(cd, state.range(0));

  for (auto _ : state) {
    benchmark::DoNotOptimize(cd.Query<SomeData1>());
  }
}

BENCHMARK(BM_CompositeData_Construct_Destruct)->Arg(5)->Arg(10)->Arg(20);
BENCHMARK(BM_CompositeData_Get_Existing)->Arg(5)->Arg(10)->Arg(20);
BENCHMARK(BM_CompositeData_Query_Existing)->Arg(5)->Arg(10)->Arg(20);

BENCHMARK_MAIN();
