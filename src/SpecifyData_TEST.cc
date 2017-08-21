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

#define IGNITION_UNITTEST_EXPECTDATA_ACCESS

#include "TestUtils.hh"
#include "ignition/physics/SpecifyData.hh"

/////////////////////////////////////////////////
// Single-requirement CompositeData
using RequireString = ignition::physics::RequireData<StringData>;

// CompositeData with three requirements and one optional expectation
using RequireStringBoolChar = ignition::physics::SpecifyData<
          ignition::physics::RequireData<
                StringData,
                BoolData,
                CharData>,
          ignition::physics::ExpectData<
                IntData> >;


/////////////////////////////////////////////////
TEST(SpecifyData, ConstructionAndAccess)
{
  // ------------------------------------------------------------------------
  // Test a CompositeData with a single requirement
  RequireString sData;

  usedExpectedDataAccess = false;
  EXPECT_TRUE(sData.Has<StringData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_TRUE(sData.Requires<StringData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  EXPECT_TRUE(sData.Expects<StringData>());
  EXPECT_TRUE(RequireString::Expects<StringData>());
  EXPECT_TRUE(sData.AlwaysRequires<StringData>());
  EXPECT_TRUE(RequireString::AlwaysRequires<StringData>());

  usedExpectedDataAccess = false;
  EXPECT_EQ("default", sData.Get<StringData>().myString);
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_EQ("default", static_cast<const RequireString&>(sData)
            .Get<StringData>().myString);


  // ------------------------------------------------------------------------
  // For each of the three required data types, make sure that they have been
  // constructed, that they are marked as required, that they have their default
  // values, and that they are being accessed using the low-cost methods.
  RequireStringBoolChar sbcData;

  usedExpectedDataAccess = false;
  EXPECT_TRUE(sbcData.Has<StringData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_TRUE(sbcData.Requires<StringData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  EXPECT_TRUE(sbcData.Expects<StringData>());
  EXPECT_TRUE(RequireStringBoolChar::Expects<StringData>());
  EXPECT_TRUE(sbcData.AlwaysRequires<StringData>());
  EXPECT_TRUE(RequireStringBoolChar::AlwaysRequires<StringData>());

  usedExpectedDataAccess = false;
  EXPECT_EQ("default", sbcData.Get<StringData>().myString);
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_EQ("default", static_cast<const RequireStringBoolChar&>(sbcData)
            .Get<StringData>().myString);
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_TRUE(sbcData.Has<BoolData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_TRUE(sbcData.Requires<BoolData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  EXPECT_TRUE(sbcData.Expects<BoolData>());
  EXPECT_TRUE(RequireStringBoolChar::Expects<BoolData>());
  EXPECT_TRUE(sbcData.AlwaysRequires<BoolData>());
  EXPECT_TRUE(RequireStringBoolChar::AlwaysRequires<BoolData>());

  usedExpectedDataAccess = false;
  EXPECT_EQ(true, static_cast<const RequireStringBoolChar&>(sbcData)
            .Get<BoolData>().myBool);
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_TRUE(sbcData.Has<CharData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_TRUE(sbcData.Requires<CharData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  EXPECT_TRUE(sbcData.Expects<CharData>());
  EXPECT_TRUE(RequireStringBoolChar::Expects<CharData>());
  EXPECT_TRUE(sbcData.AlwaysRequires<CharData>());
  EXPECT_TRUE(RequireStringBoolChar::AlwaysRequires<CharData>());

  usedExpectedDataAccess = false;
  EXPECT_EQ('c', sbcData.Get<CharData>().myChar);
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_EQ('c', static_cast<const RequireStringBoolChar&>(sbcData)
            .Get<CharData>().myChar);
  EXPECT_TRUE(usedExpectedDataAccess);


  // ------------------------------------------------------------------------
  // For the expected (but optional) data type, make sure that it was not
  // constructed, that it is not required, and that we can operate on it using
  // the low-cost methods.
  usedExpectedDataAccess = false;
  EXPECT_FALSE(sbcData.Has<IntData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_EQ(nullptr, sbcData.Query<IntData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_EQ(nullptr, static_cast<const RequireStringBoolChar&>(
              sbcData).Query<IntData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_FALSE(sbcData.Requires<IntData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  EXPECT_TRUE(sbcData.Expects<IntData>());
  EXPECT_TRUE(RequireStringBoolChar::Expects<IntData>());
  EXPECT_FALSE(sbcData.AlwaysRequires<IntData>());
  EXPECT_FALSE(RequireStringBoolChar::AlwaysRequires<IntData>());

  usedExpectedDataAccess = false;
  EXPECT_EQ(351, sbcData.GetOrCreate<IntData>(351).myInt);
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_TRUE(sbcData.Remove<IntData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_EQ(261, sbcData.Create<IntData>(261).myInt);
  EXPECT_TRUE(usedExpectedDataAccess);

  sbcData.Remove<IntData>();

  usedExpectedDataAccess = false;
  EXPECT_EQ(62, sbcData.MakeRequired<IntData>(62).myInt);
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_TRUE(sbcData.Has<IntData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  usedExpectedDataAccess = false;
  EXPECT_EQ(62, sbcData.Get<IntData>().myInt);
  EXPECT_TRUE(usedExpectedDataAccess);

  // Note: Trying to call the const-qualifed Get<IntData>() would intentionally
  // cause a compilation error, because we cannot know if it will be available
  // at compile time

  usedExpectedDataAccess = false;
  EXPECT_TRUE(sbcData.Requires<IntData>());
  EXPECT_TRUE(usedExpectedDataAccess);

  EXPECT_FALSE(sbcData.AlwaysRequires<IntData>());


  // ------------------------------------------------------------------------
  // For the unexpected data type, make sure that it was not constructed, that
  // it is not required, and that we cannot query for it using the low-cost
  // methods
  usedExpectedDataAccess = false;
  EXPECT_FALSE(sbcData.Has<DoubleData>());
  EXPECT_FALSE(usedExpectedDataAccess);

  EXPECT_EQ(nullptr, sbcData.Query<DoubleData>());
  EXPECT_FALSE(usedExpectedDataAccess);

  EXPECT_EQ(nullptr, static_cast<const RequireStringBoolChar&>(
              sbcData).Query<DoubleData>());
  EXPECT_FALSE(usedExpectedDataAccess);

  EXPECT_FALSE(sbcData.Requires<DoubleData>());
  EXPECT_FALSE(usedExpectedDataAccess);

  EXPECT_NEAR(2.45, sbcData.Create<DoubleData>(2.45).myDouble, 1e-8);
  EXPECT_FALSE(usedExpectedDataAccess);

  EXPECT_NEAR(2.45, sbcData.Get<DoubleData>().myDouble, 1e-8);
  EXPECT_FALSE(usedExpectedDataAccess);

  EXPECT_TRUE(sbcData.Remove<DoubleData>());
  EXPECT_FALSE(usedExpectedDataAccess);

  EXPECT_FALSE(sbcData.Expects<DoubleData>());
  EXPECT_FALSE(RequireStringBoolChar::Expects<DoubleData>());
  EXPECT_FALSE(sbcData.Requires<DoubleData>());
  EXPECT_FALSE(sbcData.AlwaysRequires<DoubleData>());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
