/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "ignition/physics/CompositeData.hh"
#include "utils/TestDataTypes.hh"

/////////////////////////////////////////////////
TEST(CompositeData_TEST, AddRemoveData)
{
  ignition::physics::CompositeData data;
  EXPECT_FALSE(data.Has<StringData>());

  StringData &s = data.Get<StringData>();
  EXPECT_TRUE(data.Has<StringData>());

  EXPECT_EQ("default", s.myString);
  s.myString = "modified";
  EXPECT_EQ("modified", data.Get<StringData>().myString);
  EXPECT_EQ("modified", data.GetOrCreate<StringData>().myString);

  data.Create<StringData>("new string");
  EXPECT_EQ("new string", data.Get<StringData>().myString);
  // Note that the reference s is no longer valid after Create has been called,
  // and it should no longer be used.

  data.Remove<StringData>();
  EXPECT_FALSE(data.Has<StringData>());

  EXPECT_EQ("remade", data.GetOrCreate<StringData>("remade").myString);
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, CopyMoveOperators)
{
  ignition::physics::CompositeData data =
      CreateSomeData<StringData, DoubleData, IntData>();

  EXPECT_EQ(3u, data.EntryCount());
  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.Has<DoubleData>());
  EXPECT_TRUE(data.Has<IntData>());

  ignition::physics::CompositeData emptyData;
  data = emptyData;

  EXPECT_EQ(0u, data.EntryCount());
  EXPECT_FALSE(data.Has<StringData>());
  EXPECT_FALSE(data.Has<DoubleData>());
  EXPECT_FALSE(data.Has<IntData>());
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, CopyFunction)
{
  ignition::physics::CompositeData data =
      CreateSomeData<StringData, DoubleData, IntData>();

  ignition::physics::CompositeData otherData =
      CreateSomeData<BoolData, CharData, FloatData>();

  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.Has<DoubleData>());
  EXPECT_TRUE(data.Has<IntData>());

  data.Copy(otherData);

  EXPECT_FALSE(data.Has<StringData>());
  EXPECT_FALSE(data.Has<DoubleData>());
  EXPECT_FALSE(data.Has<IntData>());

  EXPECT_TRUE(data.Has<BoolData>());
  EXPECT_TRUE(data.Has<CharData>());
  EXPECT_TRUE(data.Has<FloatData>());
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, CopyFunctionWithRequirements)
{
  ignition::physics::CompositeData data =
      CreateSomeData<StringData, DoubleData, IntData>();

  EXPECT_FALSE(data.Requires<StringData>());
  EXPECT_FALSE(data.Requires<DoubleData>());
  EXPECT_FALSE(data.Requires<IntData>());

  data.MakeRequired<DoubleData>();

  EXPECT_TRUE(data.Requires<DoubleData>());

  EXPECT_FALSE(data.Requires<StringData>());
  EXPECT_FALSE(data.Requires<IntData>());


  ignition::physics::CompositeData otherData =
      CreateSomeData<BoolData, CharData, FloatData>();
  otherData.MakeRequired<BoolData>();

  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.Has<DoubleData>());
  EXPECT_TRUE(data.Has<IntData>());

  data.Copy(otherData);

  EXPECT_TRUE(data.Has<DoubleData>());

  EXPECT_FALSE(data.Has<StringData>());
  EXPECT_FALSE(data.Has<IntData>());

  EXPECT_TRUE(data.Requires<DoubleData>());

  EXPECT_FALSE(data.Requires<StringData>());
  EXPECT_FALSE(data.Requires<IntData>());


  EXPECT_TRUE(data.Has<BoolData>());
  EXPECT_TRUE(data.Has<CharData>());
  EXPECT_TRUE(data.Has<FloatData>());

  EXPECT_FALSE(data.Requires<BoolData>());
  EXPECT_FALSE(data.Requires<CharData>());
  EXPECT_FALSE(data.Requires<IntData>());


  data.Copy(otherData, true);

  EXPECT_TRUE(data.Requires<DoubleData>());
  EXPECT_TRUE(data.Requires<BoolData>());

  EXPECT_FALSE(data.Requires<CharData>());
  EXPECT_FALSE(data.Requires<FloatData>());
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, MergeFunction)
{
  ignition::physics::CompositeData data =
      CreateSomeData<StringData, DoubleData, IntData>();

  ignition::physics::CompositeData otherData =
      CreateSomeData<BoolData, CharData, FloatData>();

  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.Has<DoubleData>());
  EXPECT_TRUE(data.Has<IntData>());

  data.Merge(otherData);

  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.Has<DoubleData>());
  EXPECT_TRUE(data.Has<IntData>());

  EXPECT_TRUE(data.Has<BoolData>());
  EXPECT_TRUE(data.Has<CharData>());
  EXPECT_TRUE(data.Has<FloatData>());
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, MergeFunctionWithRequirements)
{
  ignition::physics::CompositeData data =
      CreateSomeData<StringData, DoubleData, IntData>();

  EXPECT_FALSE(data.Requires<StringData>());
  EXPECT_FALSE(data.Requires<DoubleData>());
  EXPECT_FALSE(data.Requires<IntData>());

  data.MakeRequired<DoubleData>();

  EXPECT_TRUE(data.Requires<DoubleData>());

  EXPECT_FALSE(data.Requires<StringData>());
  EXPECT_FALSE(data.Requires<IntData>());


  ignition::physics::CompositeData otherData =
      CreateSomeData<BoolData, CharData, FloatData>();
  otherData.MakeRequired<BoolData>();

  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.Has<DoubleData>());
  EXPECT_TRUE(data.Has<IntData>());

  data.Merge(otherData);

  EXPECT_TRUE(data.Has<DoubleData>());
  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.Has<IntData>());

  EXPECT_TRUE(data.Requires<DoubleData>());

  EXPECT_FALSE(data.Requires<StringData>());
  EXPECT_FALSE(data.Requires<IntData>());


  EXPECT_TRUE(data.Has<BoolData>());
  EXPECT_TRUE(data.Has<CharData>());
  EXPECT_TRUE(data.Has<FloatData>());

  EXPECT_FALSE(data.Requires<BoolData>());
  EXPECT_FALSE(data.Requires<CharData>());
  EXPECT_FALSE(data.Requires<IntData>());


  data.Merge(otherData, true);

  EXPECT_TRUE(data.Requires<DoubleData>());
  EXPECT_TRUE(data.Requires<BoolData>());

  EXPECT_FALSE(data.Requires<CharData>());
  EXPECT_FALSE(data.Requires<FloatData>());
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, Requirements)
{
  ignition::physics::CompositeData requiredData;

  // If StringData was not already created, we should create a new one when it
  // gets marked as required, using the arguments passed in by MarkRequired
  EXPECT_FALSE(requiredData.Requires<StringData>());

  requiredData.MakeRequired<StringData>("I am required");
  EXPECT_EQ(1u, requiredData.EntryCount());
  EXPECT_EQ("I am required", requiredData.Get<StringData>().myString);
  EXPECT_TRUE(requiredData.Requires<StringData>());
  EXPECT_FALSE(requiredData.Requires<IntData>());

  // If someone asks to remove StringData, it should fail because StringData is
  // marked as required.
  EXPECT_FALSE(requiredData.Remove<StringData>());
  EXPECT_TRUE(requiredData.Has<StringData>());

  requiredData.Create<IntData>(146);
  EXPECT_EQ(146, requiredData.Get<IntData>().myInt);

  // If IntData was already created, we should not create a new one
  requiredData.MakeRequired<IntData>(641);
  EXPECT_EQ(146, requiredData.Get<IntData>().myInt);

  requiredData.Create<DoubleData>();
  EXPECT_TRUE(requiredData.Has<DoubleData>());

  // When we copy from a blank object, we should retain the required data and
  // lose everything else.
  requiredData = ignition::physics::CompositeData();
  EXPECT_TRUE(requiredData.Has<StringData>());
  EXPECT_TRUE(requiredData.Has<IntData>());
  EXPECT_FALSE(requiredData.Has<DoubleData>());
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, Queries)
{
  // Note that if we do not pass the `true` argument into CreateSomeData, then
  // the following tests fail because the compiler seems to be eliding the
  // copy/move operators and copy/move constructors, perhaps using return value
  // optimization. That gives us the wrong query behavior. If we instead did
  //
  //   ignition::physics::CompositeData data;
  //   data = CreateSomeData<StringData, DoubleData, IntData>();
  //
  // we would get the correct query behavior. I feel like this is a bug in the
  // compiler, because it should only elide those operators if it can have no
  // effect on the object's observable state. I also removed the mutable
  // declarations from the queried and numQueries flags, but this apparent bug
  // still persisted. I'm not sure what to do except declare that it should be
  // considered good practice to call ResetQueries() before returning a
  // CompositeData from a function.

  ignition::physics::CompositeData data =
      CreateSomeData<StringData, DoubleData, IntData>(true);

  std::set<std::string> unqueried, all;

  EXPECT_EQ(3u, data.UnqueriedEntryCount());
  EXPECT_EQ(3u, data.EntryCount());
  unqueried = data.UnqueriedEntries();
  EXPECT_EQ(3u, unqueried.size());
  EXPECT_NE(0u, unqueried.count(typeid(StringData).name()));
  EXPECT_NE(0u, unqueried.count(typeid(DoubleData).name()));
  EXPECT_NE(0u, unqueried.count(typeid(IntData).name()));
  all = data.AllEntries();
  EXPECT_EQ(3u, all.size());
  EXPECT_NE(0u, all.count(typeid(StringData).name()));
  EXPECT_NE(0u, all.count(typeid(DoubleData).name()));
  EXPECT_NE(0u, all.count(typeid(IntData).name()));

  data.Remove<IntData>();
  EXPECT_EQ(2u, data.UnqueriedEntryCount());
  EXPECT_EQ(2u, data.EntryCount());
  unqueried = data.UnqueriedEntries();
  EXPECT_EQ(2u, unqueried.size());
  EXPECT_NE(0u, unqueried.count(typeid(StringData).name()));
  EXPECT_NE(0u, unqueried.count(typeid(DoubleData).name()));
  EXPECT_EQ(0u, unqueried.count(typeid(IntData).name()));
  all = data.AllEntries();
  EXPECT_EQ(2u, all.size());
  EXPECT_NE(0u, all.count(typeid(StringData).name()));
  EXPECT_NE(0u, all.count(typeid(DoubleData).name()));
  EXPECT_EQ(0u, all.count(typeid(IntData).name()));

  data.Has<StringData>();
  EXPECT_EQ(2u, data.UnqueriedEntryCount());

  data.Query<StringData>();
  EXPECT_EQ(1u, data.UnqueriedEntryCount());
  EXPECT_EQ(2u, data.EntryCount());
  unqueried = data.UnqueriedEntries();
  EXPECT_EQ(1u, unqueried.size());
  EXPECT_EQ(0u, unqueried.count(typeid(StringData).name()));
  EXPECT_NE(0u, unqueried.count(typeid(DoubleData).name()));
  all = data.AllEntries();
  EXPECT_EQ(2u, all.size());
  EXPECT_NE(0u, all.count(typeid(StringData).name()));
  EXPECT_NE(0u, all.count(typeid(DoubleData).name()));


  // Objects which are newly created should be unqueried. Objects which already
  // existed will retain their previous query flags.
  data.Merge(CreateSomeData<StringData, IntData, BoolData>(true));
  EXPECT_EQ(3u, data.UnqueriedEntryCount());
  EXPECT_EQ(4u, data.EntryCount());
  unqueried = data.UnqueriedEntries();
  EXPECT_EQ(3u, unqueried.size());
  EXPECT_EQ(0u, unqueried.count(typeid(StringData).name()));
  EXPECT_NE(0u, unqueried.count(typeid(DoubleData).name()));
  EXPECT_NE(0u, unqueried.count(typeid(IntData).name()));
  EXPECT_NE(0u, unqueried.count(typeid(BoolData).name()));
  all = data.AllEntries();
  EXPECT_EQ(4u, all.size());
  EXPECT_NE(0u, all.count(typeid(StringData).name()));
  EXPECT_NE(0u, all.count(typeid(DoubleData).name()));
  EXPECT_NE(0u, all.count(typeid(IntData).name()));
  EXPECT_NE(0u, all.count(typeid(BoolData).name()));


  // Check that querying will alter the query flag
  EXPECT_NE(nullptr, data.Query<DoubleData>());
  EXPECT_EQ(2u, data.UnqueriedEntryCount());
  EXPECT_EQ(4u, data.EntryCount());
  unqueried = data.UnqueriedEntries();
  EXPECT_EQ(2u, unqueried.size());
  EXPECT_EQ(0u, unqueried.count(typeid(StringData).name()));
  EXPECT_EQ(0u, unqueried.count(typeid(DoubleData).name()));
  EXPECT_NE(0u, unqueried.count(typeid(IntData).name()));
  EXPECT_NE(0u, unqueried.count(typeid(BoolData).name()));
  all = data.AllEntries();
  EXPECT_EQ(4u, all.size());
  EXPECT_NE(0u, all.count(typeid(StringData).name()));
  EXPECT_NE(0u, all.count(typeid(DoubleData).name()));
  EXPECT_NE(0u, all.count(typeid(IntData).name()));
  EXPECT_NE(0u, all.count(typeid(BoolData).name()));


  // Make sure that the const-qualified version of query also works
  EXPECT_NE(nullptr, static_cast<const ignition::physics::CompositeData&>(
              data).Query<IntData>());
  EXPECT_EQ(1u, data.UnqueriedEntryCount());
  EXPECT_EQ(4u, data.EntryCount());
  unqueried = data.UnqueriedEntries();
  EXPECT_EQ(1u, unqueried.size());
  EXPECT_EQ(0u, unqueried.count(typeid(StringData).name()));
  EXPECT_EQ(0u, unqueried.count(typeid(DoubleData).name()));
  EXPECT_EQ(0u, unqueried.count(typeid(IntData).name()));
  EXPECT_NE(0u, unqueried.count(typeid(BoolData).name()));
  all = data.AllEntries();
  EXPECT_EQ(4u, all.size());
  EXPECT_NE(0u, all.count(typeid(StringData).name()));
  EXPECT_NE(0u, all.count(typeid(DoubleData).name()));
  EXPECT_NE(0u, all.count(typeid(IntData).name()));
  EXPECT_NE(0u, all.count(typeid(BoolData).name()));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
