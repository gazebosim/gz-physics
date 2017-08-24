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

#include <chrono>
#include <iomanip>

#include <gtest/gtest.h>

#include "utils/TestDataTypes.hh"

using ExpectStringBoolChar =
    ignition::physics::ExpectData<
        StringData,
        BoolData,
        CharData>;

using ExpectCharBoolString =
    ignition::physics::ExpectData<
        CharData,
        BoolData,
        StringData>;

using ExpectString = ignition::physics::ExpectData<StringData>;

ignition::physics::CompositeData CreatePerformanceTestData()
{
  return CreateSomeData<StringData, DoubleData, IntData,
      FloatData, VectorDoubleData, BoolData, CharData>();
}

template <typename CompositeType>
double RunPerformanceTest(CompositeType &data, const std::string &label)
{
  const std::size_t NumTests = 100000;
  const auto start = std::chrono::high_resolution_clock::now();
  for (std::size_t i=0; i < NumTests; ++i)
  {
    StringData &s = data.template Get<StringData>();
  }
  const auto finish = std::chrono::high_resolution_clock::now();

  const auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(
        finish - start).count();

  const double avg = static_cast<double>(time)/static_cast<double>(NumTests);

  std::cout << std::fixed;
  std::cout << std::setprecision(3);
  std::cout << std::right;

  std::cout << " --- " << label << " results ---\n"
            << "Avg time: "
            << std::setw(8) << std::right << avg << " ns\n"
            << std::endl;

  return avg;
}

TEST(ExpectData, AccessTime)
{
  ExpectStringBoolChar expect_sbc;
  ExpectCharBoolString expect_cbs;
  ExpectString expect_s;
  ignition::physics::CompositeData plain;

  expect_sbc.Copy(CreatePerformanceTestData());
  expect_cbs.Copy(CreatePerformanceTestData());
  expect_s.Copy(CreatePerformanceTestData());
  plain.Copy(CreatePerformanceTestData());

  const double avg_expect_sbc = RunPerformanceTest(
        expect_sbc, "Three expectations (leading)");

  const double avg_expect_cbs = RunPerformanceTest(
        expect_cbs, "Three expectations (ending)");

  const double avg_expect_s = RunPerformanceTest(
        expect_s, "One expectation");

  const double avg_plain = RunPerformanceTest(
        plain, "No expectations");

  EXPECT_LT(avg_expect_sbc, avg_plain);
  EXPECT_LT(avg_expect_cbs, avg_plain);
  EXPECT_LT(avg_expect_s, avg_plain);
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
