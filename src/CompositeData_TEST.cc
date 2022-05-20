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

#include "gz/physics/CompositeData.hh"
#include "utils/TestDataTypes.hh"

using gz::physics::CompositeData;

/////////////////////////////////////////////////
TEST(CompositeData_TEST, DestructorCoverage)
{
  // getting full destructor coverage
  CompositeData *p = new CompositeData;
  EXPECT_TRUE(p != NULL);
  delete p;
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, Get)
{
  CompositeData data;
  EXPECT_FALSE(data.Has<StringData>());
  EXPECT_FALSE(data.StatusOf<StringData>().queried);
  EXPECT_FALSE(data.Unquery<StringData>());
  EXPECT_TRUE(data.AllEntries().empty());
  EXPECT_TRUE(data.UnqueriedEntries().empty());

  StringData &s = data.Get<StringData>();
  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_EQ(1u, data.AllEntries().size());
  EXPECT_TRUE(data.UnqueriedEntries().empty());
  EXPECT_TRUE(data.Unquery<StringData>());
  EXPECT_EQ(1u, data.UnqueriedEntries().size());


  EXPECT_EQ("default", s.myString);
  s.myString = "modified";
  EXPECT_EQ("modified", data.Get<StringData>().myString);

  CompositeData other = data;
  EXPECT_TRUE(other.Has<StringData>());
  EXPECT_EQ(1u, other.UnqueriedEntries().size());
  EXPECT_FALSE(other.StatusOf<StringData>().queried);
  EXPECT_EQ("modified", other.Get<StringData>().myString);
  EXPECT_TRUE(other.UnqueriedEntries().empty());
  EXPECT_TRUE(other.StatusOf<StringData>().queried);
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, Insert)
{
  CompositeData data;
  EXPECT_FALSE(data.Has<StringData>());
  EXPECT_FALSE(data.StatusOf<StringData>().queried);

  // After the first insertion attempt, the StringData entry should come into
  // existence, constructed with the given argument.
  CompositeData::InsertResult<StringData> result =
      data.Insert<StringData>("inserted_data");
  EXPECT_TRUE(result.inserted);
  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.StatusOf<StringData>().queried);
  EXPECT_EQ("inserted_data", result.data.myString);
  // These pointers should match, because the references should refer to the
  // exact same object.
  EXPECT_EQ(&data.Get<StringData>(), &result.data);

  // After the second insertion attempt, nothing should change. The insertion
  // will have failed because an entry for StringData already existed.
  CompositeData::InsertResult<StringData> secondResult =
      data.Insert<StringData>("second_insert");
  EXPECT_FALSE(secondResult.inserted);
  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.StatusOf<StringData>().queried);
  EXPECT_NE("second_insert", secondResult.data.myString);
  EXPECT_EQ("inserted_data", secondResult.data.myString);
  // These pointers should match, because the references should refer to the
  // exact same object.
  EXPECT_EQ(&data.Get<StringData>(), &secondResult.data);
  EXPECT_EQ(&result.data, &secondResult.data);

  EXPECT_EQ(IntData().myInt, data.Insert<IntData>().data.myInt);
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, InsertOrAssign)
{
  CompositeData data;
  EXPECT_FALSE(data.Has<StringData>());
  EXPECT_FALSE(data.StatusOf<StringData>().queried);

  // After the first InsertOrAssign attempt, the StringData entry should come
  // into existence, constructed with the given argument.
  CompositeData::InsertResult<StringData> result =
      data.InsertOrAssign<StringData>("inserted_data");
  EXPECT_TRUE(result.inserted);
  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.StatusOf<StringData>().queried);
  EXPECT_EQ("inserted_data", result.data.myString);
  // These pointers should match, because the references should refer to the
  // exact same object.
  EXPECT_EQ(&data.Get<StringData>(), &result.data);

  // After the second InsertOrAssign attempt, the existing StringData entry
  // should be assigned the new value while remaining the same physical object
  // in memory.
  CompositeData::InsertResult<StringData> secondResult =
      data.InsertOrAssign<StringData>("assigned_data");
  EXPECT_FALSE(secondResult.inserted);
  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.StatusOf<StringData>().queried);
  EXPECT_EQ("assigned_data", secondResult.data.myString);
  // These pointers should match, because the references should refer to the
  // exact same object. Also, the underlying object should still be the same
  // one in physical memory, even though its value has changed.
  EXPECT_EQ(&data.Get<StringData>(), &secondResult.data);
  EXPECT_EQ(&result.data, &secondResult.data);
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, CopyMoveOperators)
{
  gz::physics::CompositeData data =
      CreateSomeData<StringData, DoubleData, IntData>();

  EXPECT_EQ(3u, data.EntryCount());
  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.Has<DoubleData>());
  EXPECT_TRUE(data.Has<IntData>());

  gz::physics::CompositeData emptyData;
  data = emptyData;

  EXPECT_EQ(0u, data.EntryCount());
  EXPECT_FALSE(data.Has<StringData>());
  EXPECT_FALSE(data.Has<DoubleData>());
  EXPECT_FALSE(data.Has<IntData>());
}

/////////////////////////////////////////////////
struct zzzzzzzzz
{
  // With the Itanium ABI the typeid(~).name() of this class will be 9zzzzzzzzz
  // In MSVC the typeid(~).name() of this class will be "class zzzzzzzzz"

  // These properties should effectively guarantee that this class is always the
  // last entry in a CompositeData instance, which is a useful property for
  // achieving complete implementation line coverage in the next unit test.
};

/////////////////////////////////////////////////
TEST(CompositeData_TEST, CopyFunction)
{
  gz::physics::CompositeData data =
      CreateSomeData<StringData, DoubleData, IntData>();

  gz::physics::CompositeData otherData =
      CreateSomeData<BoolData, CharData, FloatData>();

  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.Has<DoubleData>());
  EXPECT_TRUE(data.Has<IntData>());

  data.Copy(otherData);

  EXPECT_FALSE(data.Has<StringData>());
  EXPECT_FALSE(data.Has<DoubleData>());
  EXPECT_FALSE(data.Has<IntData>());

  EXPECT_TRUE(data.Has<BoolData>());
  EXPECT_EQ(otherData.Get<BoolData>().myBool, data.Get<BoolData>().myBool);

  EXPECT_TRUE(data.Has<CharData>());
  EXPECT_EQ(otherData.Get<CharData>().myChar, data.Get<CharData>().myChar);

  EXPECT_TRUE(data.Has<FloatData>());
  EXPECT_NEAR(otherData.Get<FloatData>().myFloat,
              data.Get<FloatData>().myFloat, 1e-8f);

  // This next section is used for implementation line coverage, to ensure that
  // StandardDataClone gets called in the implementation.
  EXPECT_TRUE(data.Remove<CharData>());
  EXPECT_FALSE(data.Has<CharData>());
  EXPECT_FALSE(data.Unquery<CharData>());

  data.Copy(otherData);

  EXPECT_TRUE(data.Has<CharData>());
  EXPECT_EQ(otherData.Get<CharData>().myChar, data.Get<CharData>().myChar);

  // The next section is used for implementation line coverage to ensure that
  // we can correctly insert data entries in the center of the data map.
  gz::physics::CompositeData zzzData = CreateSomeData<zzzzzzzzz>();
  EXPECT_TRUE(zzzData.Has<zzzzzzzzz>());
  zzzData.Copy(otherData);

  EXPECT_TRUE(zzzData.Has<BoolData>());
  EXPECT_TRUE(zzzData.Has<CharData>());
  EXPECT_TRUE(zzzData.Has<FloatData>());
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, CopyFunctionWithRequirements)
{
  gz::physics::CompositeData data =
      CreateSomeData<StringData, DoubleData, IntData>();

  EXPECT_FALSE(data.Requires<StringData>());
  EXPECT_FALSE(data.Requires<DoubleData>());
  EXPECT_FALSE(data.Requires<IntData>());

  data.MakeRequired<DoubleData>();

  EXPECT_TRUE(data.Requires<DoubleData>());

  EXPECT_FALSE(data.Requires<StringData>());
  EXPECT_FALSE(data.Requires<IntData>());


  gz::physics::CompositeData otherData =
      CreateSomeData<BoolData, CharData, FloatData>();
  otherData.MakeRequired<BoolData>();

  // Inserting and then removing this data entry helps with our implementation
  // line coverage
  EXPECT_TRUE(otherData.Insert<StringData>().inserted);
  EXPECT_TRUE(otherData.Remove<StringData>());

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

  // Removing this data entry here helps to complete line coverage
  EXPECT_TRUE(data.Remove<BoolData>());
  EXPECT_FALSE(data.Has<BoolData>());
  EXPECT_FALSE(data.Unquery<BoolData>());

  data.Copy(otherData, true);

  EXPECT_TRUE(data.Requires<DoubleData>());
  EXPECT_TRUE(data.Requires<BoolData>());

  EXPECT_FALSE(data.Requires<CharData>());
  EXPECT_FALSE(data.Requires<FloatData>());

  // This next segment helps with line coverage in the implementation
  otherData.MakeRequired<FloatData>();
  EXPECT_TRUE(data.Remove<FloatData>());
  EXPECT_FALSE(data.Has<FloatData>());
  EXPECT_FALSE(data.Requires<FloatData>());
  data.Copy(std::move(otherData), true);

  EXPECT_TRUE(data.Has<FloatData>());
  EXPECT_TRUE(data.Requires<FloatData>());
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, MergeFunction)
{
  gz::physics::CompositeData data =
      CreateSomeData<StringData, DoubleData, IntData>();

  gz::physics::CompositeData otherData =
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
  gz::physics::CompositeData data =
      CreateSomeData<StringData, DoubleData, IntData>();

  EXPECT_FALSE(data.Requires<StringData>());
  EXPECT_FALSE(data.Requires<DoubleData>());
  EXPECT_FALSE(data.Requires<IntData>());

  data.MakeRequired<DoubleData>();

  EXPECT_TRUE(data.Requires<DoubleData>());

  EXPECT_FALSE(data.Requires<StringData>());
  EXPECT_FALSE(data.Requires<IntData>());


  gz::physics::CompositeData otherData =
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
TEST(CompositeData_TEST, Remove)
{
  gz::physics::CompositeData data;

  // try to remove data from an empty container
  // it should return true because the container does not have it now
  // (even though it never did)
  EXPECT_FALSE(data.Has<StringData>());
  EXPECT_FALSE(data.Has<DoubleData>());
  EXPECT_FALSE(data.Has<IntData>());
  EXPECT_TRUE(data.Remove<StringData>());
  EXPECT_TRUE(data.Remove<DoubleData>());
  EXPECT_TRUE(data.Remove<IntData>());

  data = CreateSomeData<StringData, DoubleData, IntData>(true);
  EXPECT_TRUE(data.Has<StringData>());
  EXPECT_TRUE(data.Has<DoubleData>());
  EXPECT_TRUE(data.Has<IntData>());
  EXPECT_NE(nullptr, data.Query<StringData>());
  EXPECT_TRUE(data.StatusOf<StringData>().exists);

  EXPECT_NE(nullptr, data.Query<IntData>());

  EXPECT_TRUE(data.Remove<StringData>());
  EXPECT_TRUE(data.Remove<DoubleData>());
  EXPECT_TRUE(data.Remove<IntData>());

  EXPECT_FALSE(data.Has<StringData>());
  EXPECT_FALSE(data.Has<DoubleData>());
  EXPECT_FALSE(data.Has<IntData>());
  EXPECT_EQ(nullptr, data.Query<StringData>());
  EXPECT_FALSE(data.StatusOf<StringData>().exists);
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, Requirements)
{
  gz::physics::CompositeData requiredData;

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

  requiredData.InsertOrAssign<IntData>(146);
  EXPECT_EQ(146, requiredData.Get<IntData>().myInt);

  // If IntData was already created, we should not create a new one
  requiredData.MakeRequired<IntData>(641);
  EXPECT_EQ(146, requiredData.Get<IntData>().myInt);

  requiredData.InsertOrAssign<DoubleData>(DoubleData());
  EXPECT_TRUE(requiredData.Has<DoubleData>());

  // When we copy from a blank object, we should retain the required data and
  // lose everything else.
  requiredData = gz::physics::CompositeData();
  EXPECT_TRUE(requiredData.Has<StringData>());
  EXPECT_TRUE(requiredData.Has<IntData>());
  EXPECT_FALSE(requiredData.Has<DoubleData>());
}

/////////////////////////////////////////////////
TEST(CompositeData_TEST, Queries)
{
  // test queries on empty container
  {
    gz::physics::CompositeData data;
    EXPECT_EQ(nullptr, data.Query<StringData>());
    EXPECT_EQ(nullptr, data.Query<DoubleData>());
    EXPECT_EQ(nullptr, data.Query<IntData>());
  }

  // Note that if we do not pass the `true` argument into CreateSomeData, then
  // the following tests fail because the compiler seems to be eliding the
  // copy/move operators and copy/move constructors, perhaps using return value
  // optimization. That gives us the wrong query behavior. If we instead did
  //
  //   gz::physics::CompositeData data;
  //   data = CreateSomeData<StringData, DoubleData, IntData>();
  //
  // we would get the correct query behavior. I feel like this is a bug in the
  // compiler, because it should only elide those operators if it can have no
  // effect on the object's observable state. I also removed the mutable
  // declarations from the queried and numQueries flags, but this apparent bug
  // still persisted. I'm not sure what to do except declare that it should be
  // considered good practice to call ResetQueries() before returning a
  // CompositeData from a function.

  gz::physics::CompositeData data =
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

  // Query a variable before removing it
  EXPECT_NE(nullptr, data.Query<IntData>());
  EXPECT_EQ(2u, data.UnqueriedEntryCount());
  EXPECT_EQ(3u, data.EntryCount());

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
  EXPECT_NE(nullptr, static_cast<const gz::physics::CompositeData&>(
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
