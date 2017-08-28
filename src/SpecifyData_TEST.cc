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

#include "utils/TestDataTypes.hh"
#include "ignition/physics/SpecifyData.hh"
#include "ignition/physics/CanReadData.hh"
#include "ignition/physics/CanWriteData.hh"


/////////////////////////////////////////////////
TEST(SpecifyData, RequirementsAccessConstruction)
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
TEST(SpecifyData, QueryCounting)
{
  const ignition::physics::CompositeData::QueryMode normal =
      ignition::physics::CompositeData::QUERY_NORMAL;

  const ignition::physics::CompositeData::QueryMode silent =
      ignition::physics::CompositeData::QUERY_SILENT;


  RequireStringBoolChar data;

  EXPECT_EQ(3u, data.NumEntries());
  EXPECT_EQ(3u, data.NumUnqueriedEntries());

  // Test a silent query on an existing expected type
  usedExpectedDataAccess = false;
  EXPECT_NE(nullptr, data.Query<StringData>(silent));
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(3u, data.NumUnqueriedEntries());

  // Test a normal query on an existing expected type
  usedExpectedDataAccess = false;
  EXPECT_NE(nullptr, data.Query<StringData>(normal));
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test a normal query on a non-existent expected type
  usedExpectedDataAccess = false;
  EXPECT_EQ(nullptr, data.Query<IntData>());
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(3u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test Create on a non-existent expected type
  usedExpectedDataAccess = false;
  EXPECT_EQ(248, data.Create<IntData>(248).myInt);
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(4u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test a normal query on a newly existing expected type
  usedExpectedDataAccess = false;
  EXPECT_EQ(nullptr, data.Query<DoubleData>(normal));
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(4u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test Create on an existing expected type
  usedExpectedDataAccess = false;
  EXPECT_EQ(272, data.Create<IntData>(272).myInt);
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(4u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test Create on a non-existent unexpected type
  usedExpectedDataAccess = false;
  EXPECT_NEAR(2.66, data.Create<DoubleData>(2.66).myDouble, 1e-8);
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(5u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test a normal query on a newly existing unexpected type
  usedExpectedDataAccess = false;
  EXPECT_NE(nullptr, data.Query<DoubleData>(normal));
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test Create on an existing unexpected type
  usedExpectedDataAccess = false;
  EXPECT_NEAR(2.92, data.Create<DoubleData>(2.92).myDouble, 1e-8);
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(5u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test unquery on an expected type
  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Unquery<IntData>());
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(3u, data.NumUnqueriedEntries());

  // Test a redundant unquery on an expected type
  usedExpectedDataAccess = false;
  EXPECT_FALSE(data.Unquery<IntData>());
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(3u, data.NumUnqueriedEntries());

  // Test remove on a queried expected type
  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Remove<IntData>());
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(4u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test unquery on an unexpected type
  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Unquery<DoubleData>());
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(3u, data.NumUnqueriedEntries());

  // Test a redundant unquery on an unexpected type
  usedExpectedDataAccess = false;
  EXPECT_FALSE(data.Unquery<DoubleData>());
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(3u, data.NumUnqueriedEntries());

  // Test remove on a queried unexpected type
  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Remove<DoubleData>());
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(3u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test GetOrCreate on an expected type
  usedExpectedDataAccess = false;
  EXPECT_EQ(299, data.GetOrCreate<IntData>(299).myInt);
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(4u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test GetOrCreate on an unexpected type
  usedExpectedDataAccess = false;
  EXPECT_NEAR(3.05, data.GetOrCreate<DoubleData>(3.05).myDouble, 1e-8);
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(5u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test Remove on an unqueried expected type
  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Unquery<IntData>());
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(3u, data.NumUnqueriedEntries());
  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Remove<IntData>());
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(4u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test Remove on an unqueried unexpected type
  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Unquery<DoubleData>());
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(3u, data.NumUnqueriedEntries());
  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Remove<DoubleData>());
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(3u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test MakeRequired on a non-existent expected type
  usedExpectedDataAccess = false;
  EXPECT_EQ(331, data.MakeRequired<IntData>(331).myInt);
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(4u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test MakeRequired on a non-existent unexpected type
  usedExpectedDataAccess = false;
  EXPECT_NEAR(3.37, data.MakeRequired<DoubleData>(3.37).myDouble, 1e-8);
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(5u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test ResetQueries
  data.ResetQueries();
  EXPECT_EQ(5u, data.NumUnqueriedEntries());

  // Test a silent Has on an expected type
  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Has<IntData>(silent));
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(5u, data.NumUnqueriedEntries());

  // Test a normal Has on an expected type
  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Has<IntData>(normal));
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(4u, data.NumUnqueriedEntries());

  // Test a silent Has on an unexpected type
  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Has<DoubleData>(silent));
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(4u, data.NumUnqueriedEntries());

  // Test a normal Has on an unexpected type
  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Has<DoubleData>(normal));
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(3u, data.NumUnqueriedEntries());

  // Test Get on an existing unqueried expected type
  usedExpectedDataAccess = false;
  data.Get<StringData>().myString = "new_string";
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test Get on an existing queried expected type
  usedExpectedDataAccess = false;
  EXPECT_EQ("new_string", data.Get<StringData>().myString);
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  EXPECT_TRUE(data.Unquery<DoubleData>());
  EXPECT_EQ(3u, data.NumUnqueriedEntries());
  // Test Get on an existing unqueried unexpected type
  usedExpectedDataAccess = false;
  EXPECT_NEAR(3.37, data.Get<DoubleData>().myDouble, 1e-8);
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test Get on an existing queried unexpected type
  usedExpectedDataAccess = false;
  EXPECT_NEAR(3.37, data.Get<DoubleData>().myDouble, 1e-8);
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test Get on a non-existent expected type
  usedExpectedDataAccess = false;
  EXPECT_NEAR(9.5, data.Get<FloatData>().myFloat, 1e-8);
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(6u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test Get on a non-existent unexpected type
  usedExpectedDataAccess = false;
  EXPECT_EQ(0u, data.Get<VectorDoubleData>().myVector.size());
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(7u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test normal StatusOf on an existing queried required type
  usedExpectedDataAccess = false;
  ignition::physics::CompositeData::DataStatus status =
      data.StatusOf<StringData>(normal);
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_TRUE(status.exists);
  EXPECT_TRUE(status.required);
  EXPECT_TRUE(status.queried);
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  EXPECT_TRUE(data.Unquery<FloatData>());
  EXPECT_EQ(3u, data.NumUnqueriedEntries());
  // Test silent StatusOf on an existing unqueried expected type
  usedExpectedDataAccess = false;
  status = data.StatusOf<FloatData>(silent);
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_TRUE(status.exists);
  EXPECT_FALSE(status.required);
  EXPECT_FALSE(status.queried);
  EXPECT_EQ(3u, data.NumUnqueriedEntries());

  // Test normal StatusOf on an existing unqueried expected type
  usedExpectedDataAccess = false;
  status = data.StatusOf<FloatData>(normal);
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_TRUE(status.exists);
  EXPECT_FALSE(status.required);
  // Note that the queried flag in a status output refers to whether it was
  // queried BEFORE the call to StatusOf.
  EXPECT_FALSE(status.queried);
  EXPECT_EQ(2u, data.NumUnqueriedEntries());
  EXPECT_TRUE(data.StatusOf<FloatData>(silent).queried);

  EXPECT_TRUE(data.Unquery<VectorDoubleData>());
  EXPECT_EQ(3u, data.NumUnqueriedEntries());
  // Test silent StatusOf on an existing unqueried unexpected type
  usedExpectedDataAccess = false;
  status = data.StatusOf<VectorDoubleData>(silent);
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_TRUE(status.exists);
  EXPECT_FALSE(status.required);
  EXPECT_FALSE(status.queried);
  EXPECT_EQ(3u, data.NumUnqueriedEntries());

  // Test normal StatusOf on an existing unqueried unexpected type
  usedExpectedDataAccess = false;
  status = data.StatusOf<VectorDoubleData>(normal);
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_TRUE(status.exists);
  EXPECT_FALSE(status.required);
  EXPECT_FALSE(status.queried);
  EXPECT_EQ(2u, data.NumUnqueriedEntries());
  EXPECT_TRUE(data.StatusOf<VectorDoubleData>(silent).queried);

  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Remove<FloatData>());
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_EQ(6u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());
  // Test silent StatusOf on a non-existent expected type
  usedExpectedDataAccess = false;
  status = data.StatusOf<FloatData>(silent);
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_FALSE(status.exists);
  EXPECT_FALSE(status.required);
  EXPECT_FALSE(status.queried);
  EXPECT_EQ(6u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test normal StatusOf on a non-existent expected type
  usedExpectedDataAccess = false;
  status = data.StatusOf<FloatData>(normal);
  EXPECT_TRUE(usedExpectedDataAccess);
  EXPECT_FALSE(status.exists);
  EXPECT_FALSE(status.required);
  EXPECT_FALSE(status.queried);
  EXPECT_EQ(6u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  usedExpectedDataAccess = false;
  EXPECT_TRUE(data.Remove<VectorDoubleData>());
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_EQ(5u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());
  // Test silent StatusOf on a non-existent unexpected type
  usedExpectedDataAccess = false;
  status = data.StatusOf<VectorDoubleData>(silent);
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_FALSE(status.exists);
  EXPECT_FALSE(status.required);
  EXPECT_FALSE(status.queried);
  EXPECT_EQ(5u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());

  // Test normal StatusOf on a non-existent unexpected type
  usedExpectedDataAccess = false;
  status = data.StatusOf<VectorDoubleData>(silent);
  EXPECT_FALSE(usedExpectedDataAccess);
  EXPECT_FALSE(status.exists);
  EXPECT_FALSE(status.required);
  EXPECT_FALSE(status.queried);
  EXPECT_EQ(5u, data.NumEntries());
  EXPECT_EQ(2u, data.NumUnqueriedEntries());
}

TEST(SpecifyData, CountData)
{
  EXPECT_EQ(1u, ignition::physics::CountUpperLimitOfRequiredData<
            RequireString>());

  EXPECT_EQ(1u, ignition::physics::CountUpperLimitOfExpectedData<
            RequireString>());

  EXPECT_EQ(3u, ignition::physics::CountUpperLimitOfRequiredData<
            RequireStringBoolChar>());

  EXPECT_EQ(5u, ignition::physics::CountUpperLimitOfExpectedData<
            RequireStringBoolChar>());

  // Note that there are only 3 unique requirements and 5 unique expectations
  // in the redundant specification, but because of limitions in the abilities
  // of constexpr, we only provide an upper limit of the count which will count
  // repeated data specifications once for each repeat. In the future, if we can
  // find a way to push these tests down to 3u and 5u, that would be ideal.
  EXPECT_EQ(4u, ignition::physics::CountUpperLimitOfRequiredData<
            RedundantSpec>());

  EXPECT_EQ(6u, ignition::physics::CountUpperLimitOfExpectedData<
            RedundantSpec>());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
