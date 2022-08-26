#include <benchmark/benchmark.h>

#include "utils/TestDataTypes.hh"

std::size_t gNumTests = 100000;

struct SomeData1 { };
struct SomeData2 { };
struct SomeData3 { };
struct SomeData4 { };
struct SomeData5 { };
struct SomeData6 { };
struct SomeData7 { };
struct SomeData8 { };
struct SomeData9 { };
struct SomeData10 { };
struct SomeData11 { };
struct SomeData12 { };
struct SomeData13 { };
struct SomeData14 { };
struct SomeData15 { };
struct SomeData16 { };
struct SomeData17 { };
struct SomeData18 { };
struct SomeData19 { };

// Expect only 1 type
using ExpectString = ignition::physics::ExpectData<StringData>;

// Expect 3 different types, and put the type we care about first in the list
using Expect3Types_Leading =
    ignition::physics::ExpectData<StringData, BoolData, CharData>;

// Expect 3 different types, and put the type we care about last in the list
using Expect3Types_Trailing =
    ignition::physics::ExpectData<CharData, BoolData, StringData>;

// Expect 10 different types, and put the type we care about first in the list
using Expect10Types_Leading =
    ignition::physics::ExpectData<
        StringData,
        SomeData1, SomeData2, SomeData3, SomeData4, SomeData5,
        SomeData6, SomeData7, SomeData8, SomeData9>;

// Expect 10 different types, and put the type we care about last in the list
using Expect10Types_Trailing =
    ignition::physics::ExpectData<
        SomeData1, SomeData2, SomeData3, SomeData4, SomeData5,
        SomeData6, SomeData7, SomeData8, SomeData9,
        StringData>;

// Expect 20 different types, and put the type we care about first in the list
using Expect20Types_Leading =
    ignition::physics::ExpectData<
        StringData,
        SomeData1, SomeData2, SomeData3, SomeData4, SomeData5,
        SomeData6, SomeData7, SomeData8, SomeData9, SomeData10,
        SomeData11, SomeData12, SomeData13, SomeData14, SomeData15,
        SomeData16, SomeData17, SomeData18, SomeData19>;

// Expect 20 different types, and put the type we care about last in the list
using Expect20Types_Trailing =
    ignition::physics::ExpectData<
        SomeData1, SomeData2, SomeData3, SomeData4, SomeData5,
        SomeData6, SomeData7, SomeData8, SomeData9, SomeData10,
        SomeData11, SomeData12, SomeData13, SomeData14, SomeData15,
        SomeData16, SomeData17, SomeData18, SomeData19,
        StringData>;

ignition::physics::CompositeData CreatePerformanceTestData()
{
  return CreateSomeData<StringData, DoubleData, IntData,
      FloatData, VectorDoubleData, BoolData, CharData>();
}

// NaiveCompositionBase and NaiveComposition are used to produce a reference
// performance result that can help put the CompositeData performance in
// perspective.
class NaiveCompositionBase
{
};

template<typename T>
// cppcheck-suppress noConstructor
class NaiveComposition : public NaiveCompositionBase
{
  public: const T &Get() const { return d; }
  private: T d;
};

template <class Q>
// NOLINTNEXTLINE
void BM_Expect(benchmark::State& _st)
{
  size_t numTests = _st.range(0);
  Q expect;
  expect.Copy(CreatePerformanceTestData());

  for (auto _ : _st)
  {
    for (std::size_t i=0; i < numTests; ++i)
    {
      expect.template Get<StringData>();
    }
  }
}

// NOLINTNEXTLINE
void BM_Naive(benchmark::State& _st)
{
  size_t numTests = _st.range(0);

  std::vector<NaiveCompositionBase*> basicComposition;
  for (std::size_t j = 0; j < numTests; ++j)
  {
    if (j < numTests / 2)
      basicComposition.push_back(new NaiveComposition<int>());
    else
      basicComposition.push_back(new NaiveComposition<std::string>());
  }

  for (auto _ : _st)
  {
    for (std::size_t j = 0; j < numTests ; ++j)
    {
      if (j < numTests/ 2)
        static_cast<NaiveComposition<int>*>(basicComposition[j])->Get();
      else
        static_cast<NaiveComposition<std::string>*>(basicComposition[j])->Get();
    }
  }
}

// NOLINTNEXTLINE
BENCHMARK(BM_Naive)->Arg(gNumTests);
// NOLINTNEXTLINE
BENCHMARK_TEMPLATE(BM_Expect, ExpectString)->Arg(gNumTests);
// NOLINTNEXTLINE
BENCHMARK_TEMPLATE(BM_Expect, Expect3Types_Leading)->Arg(gNumTests);
// NOLINTNEXTLINE
BENCHMARK_TEMPLATE(BM_Expect, Expect3Types_Trailing)->Arg(gNumTests);
// NOLINTNEXTLINE
BENCHMARK_TEMPLATE(BM_Expect, Expect10Types_Leading)->Arg(gNumTests);
// NOLINTNEXTLINE
BENCHMARK_TEMPLATE(BM_Expect, Expect10Types_Trailing)->Arg(gNumTests);
// NOLINTNEXTLINE
BENCHMARK_TEMPLATE(BM_Expect, Expect20Types_Leading)->Arg(gNumTests);
// NOLINTNEXTLINE
BENCHMARK_TEMPLATE(BM_Expect, Expect20Types_Trailing)->Arg(gNumTests);
// NOLINTNEXTLINE
BENCHMARK_TEMPLATE(BM_Expect, ignition::physics::CompositeData)->Arg(gNumTests);

// OSX needs the semicolon, Ubuntu complains that there's an extra ';'
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
BENCHMARK_MAIN();
#pragma GCC diagnostic pop
