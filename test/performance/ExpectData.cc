/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>

#include <chrono>
#include <iomanip>
#include <cmath>

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

template <typename CompositeType>
double RunPerformanceTest(CompositeType &data)
{
  const auto start = std::chrono::high_resolution_clock::now();
  for (std::size_t i=0; i < gNumTests; ++i)
  {
    data.template Get<StringData>();
  }
  const auto finish = std::chrono::high_resolution_clock::now();

  const auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(
        finish - start).count();

  const double avg = static_cast<double>(time)/static_cast<double>(gNumTests);

  return avg;
}

// NaiveCompositionBase and NaiveComposition are used to produce a reference
// performance result that can help put the CompositeData performance in
// perspective.
class NaiveCompositionBase
{
};

template<typename T>
class NaiveComposition : public NaiveCompositionBase
{
  public: const T &Get() const { return d; }
  private: T d;
};


TEST(ExpectData, AccessTime)
{
  ExpectString expect_1;
  Expect3Types_Leading expect_3_leading;
  Expect3Types_Trailing expect_3_trailing;
  Expect10Types_Leading expect_10_leading;
  Expect10Types_Trailing expect_10_trailing;
  Expect20Types_Leading expect_20_leading;
  Expect20Types_Trailing expect_20_trailing;
  ignition::physics::CompositeData plain;

  expect_1.Copy(CreatePerformanceTestData());
  expect_3_leading.Copy(CreatePerformanceTestData());
  expect_3_trailing.Copy(CreatePerformanceTestData());
  expect_10_leading.Copy(CreatePerformanceTestData());
  expect_10_trailing.Copy(CreatePerformanceTestData());
  expect_20_leading.Copy(CreatePerformanceTestData());
  expect_20_trailing.Copy(CreatePerformanceTestData());
  plain.Copy(CreatePerformanceTestData());

  double avg_expect_1 = 0.0;
  double avg_expect_3_leading = 0.0;
  double avg_expect_3_trailing = 0.0;
  double avg_expect_10_leading = 0.0;
  double avg_expect_10_trailing = 0.0;
  double avg_expect_20_leading = 0.0;
  double avg_expect_20_trailing = 0.0;
  double avg_plain = 0.0;
  double avg_naive = 0.0;

  const std::size_t NumRuns = 100;

  std::vector<NaiveCompositionBase*> basicComposition;
  for (std::size_t j = 0; j < gNumTests; ++j)
  {
    if (j < gNumTests / 2)
      basicComposition.push_back(new NaiveComposition<int>());
    else
      basicComposition.push_back(new NaiveComposition<std::string>());
  }

  for (std::size_t i = 0; i < NumRuns; ++i)
  {
    avg_expect_1 += RunPerformanceTest(expect_1);
    avg_expect_3_leading += RunPerformanceTest(expect_3_leading);
    avg_expect_3_trailing += RunPerformanceTest(expect_3_trailing);
    avg_expect_10_leading += RunPerformanceTest(expect_10_leading);
    avg_expect_10_trailing += RunPerformanceTest(expect_10_trailing);
    avg_expect_20_leading += RunPerformanceTest(expect_20_leading);
    avg_expect_20_trailing += RunPerformanceTest(expect_20_trailing);
    avg_plain += RunPerformanceTest(plain);

    const auto start = std::chrono::high_resolution_clock::now();
    for (std::size_t j = 0; j < gNumTests; ++j)
    {
      if (j < gNumTests / 2)
        static_cast<NaiveComposition<int>*>(basicComposition[j])->Get();
      else
        static_cast<NaiveComposition<std::string>*>(basicComposition[j])->Get();
    }
    const auto finish = std::chrono::high_resolution_clock::now();
    const auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(
        finish - start).count();

    avg_naive += static_cast<double>(time)/static_cast<double>(gNumTests);
  }

  EXPECT_LT(avg_expect_1, avg_plain);
  EXPECT_LT(avg_expect_3_leading, avg_plain);
  EXPECT_LT(avg_expect_3_trailing, avg_plain);
  EXPECT_LT(avg_expect_10_leading, avg_plain);
  EXPECT_LT(avg_expect_10_trailing, avg_plain);
  EXPECT_LT(avg_expect_20_leading, avg_plain);
  EXPECT_LT(avg_expect_20_trailing, avg_plain);

  // These 6 results should be very close to each other. avg_expect_1 is an
  // exception because it uses the plain ExpectData<T> instead of being wrapped
  // in a SpecifyData<T> which adds just a tiny bit of overhead because it needs
  // to make one additional function call.
  const double baseline = avg_expect_3_leading;
  EXPECT_LT(std::abs(avg_expect_3_trailing - baseline), baseline);
  EXPECT_LT(std::abs(avg_expect_10_leading - baseline), baseline);
  EXPECT_LT(std::abs(avg_expect_10_trailing - baseline), baseline);
  EXPECT_LT(std::abs(avg_expect_20_leading - baseline), baseline);
  EXPECT_LT(std::abs(avg_expect_20_trailing - baseline), baseline);

  std::vector<std::string> labels = {
    "1 expectation",
    "3 expectations (leading)",
    "3 expectations (ending)",
    "10 expectations (leading)",
    "10 expectations (trailing)",
    "20 expectations (leading)",
    "20 expectations (trailing)",
    "No expectations",
    "Naive composition: This is only a reference point" };

  std::vector<double> avgs = {
    avg_expect_1,
    avg_expect_3_leading,
    avg_expect_3_trailing,
    avg_expect_10_leading,
    avg_expect_10_trailing,
    avg_expect_20_leading,
    avg_expect_20_trailing,
    avg_plain,
    avg_naive };

  for (std::size_t i = 0; i < labels.size(); ++i)
  {
    std::cout << std::fixed;
    std::cout << std::setprecision(6);
    std::cout << std::right;

    std::cout << " --- " << labels[i] << " result ---\n"
              << "Avg time: " << std::setw(8) << std::right
              << avgs[i]/static_cast<double>(NumRuns)
              << " ns\n" << std::endl;
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
