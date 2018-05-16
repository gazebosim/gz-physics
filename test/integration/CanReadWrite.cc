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

#include "ignition/physics/SpecifyData.hh"
#include "ignition/physics/CanReadData.hh"
#include "ignition/physics/CanWriteData.hh"

#include "utils/TestDataTypes.hh"

using ignition::physics::CanReadRequiredData;
using ignition::physics::CanReadExpectedData;
using ignition::physics::CanWriteExpectedData;
using ignition::physics::CanWriteRequiredData;

template <typename ReadSpec>
class SomeClassBase
{
  public: StringData sdata;
  public: std::size_t scount;

  public: BoolData bdata;
  public: std::size_t bcount;

  public: CharData cdata;
  public: std::size_t ccount;

  public: IntData idata;
  public: std::size_t icount;

  public: FloatData fdata;
  public: std::size_t fcount;

  SomeClassBase()
    : scount(0),
      bcount(0),
      ccount(0),
      icount(0),
      fcount(0)
  {
    // Do nothing
  }

  public: void Read(const StringData& _sdata)
  {
    sdata = _sdata;
    ++scount;
  }

  public: void Read(const BoolData& _bdata)
  {
    bdata = _bdata;
    ++bcount;
  }

  public: void Read(const CharData& _cdata)
  {
    cdata = _cdata;
    ++ccount;
  }

  public: void Read(const IntData& _idata)
  {
    idata = _idata;
    ++icount;
  }

  public: void Read(const FloatData& _fdata)
  {
    fdata = _fdata;
    ++fcount;
  }

  public: void Write(IntData& _idata) const
  {
    _idata.myInt = 67;
  }

  public: void Write(DoubleData& _ddata) const
  {
    _ddata.myDouble = 7.2;
  }

  public: void Write(StringData& _sdata) const
  {
    _sdata.myString = "seventy-seven";
  }

  public: void Write(CharData& _cdata) const
  {
    _cdata.myChar = '8';
  }
};

template <typename ReadSpec>
class SomeClass
    : public SomeClassBase<ReadSpec>,
      public CanReadRequiredData<SomeClass<ReadSpec>, ReadSpec>,
      public CanReadExpectedData<SomeClass<ReadSpec>, ReadSpec>,
      public CanWriteExpectedData<SomeClass<ReadSpec>, RequireIntDouble>
{
  public: SomeClass()
    : SomeClassBase<ReadSpec>()
  {
    // Do nothing
  }
};

/////////////////////////////////////////////////
TEST(CanReadWrite, ReadWriteData)
{
  ignition::physics::CompositeData input;
  input.Get<StringData>().myString = "89";
  input.Get<BoolData>().myBool = false;
  input.Get<CharData>().myChar = 'd';
  input.Get<IntData>().myInt = 92;
  input.Get<FloatData>().myFloat = 93.5;
  input.ResetQueries();

  SomeClass<RequireStringBoolChar> something;
  something.ReadRequiredData(input);
  EXPECT_EQ("89", something.sdata.myString);
  EXPECT_FALSE(something.bdata.myBool);
  EXPECT_EQ('d', something.cdata.myChar);
  EXPECT_EQ(55, something.idata.myInt);
  EXPECT_FLOAT_EQ(9.5, something.fdata.myFloat);

  something.ReadExpectedData(input);
  EXPECT_EQ(92, something.idata.myInt);
  EXPECT_FLOAT_EQ(93.5, something.fdata.myFloat);

  EXPECT_EQ(1u, something.scount);
  EXPECT_EQ(1u, something.bcount);
  EXPECT_EQ(1u, something.ccount);
  EXPECT_EQ(1u, something.icount);
  EXPECT_EQ(1u, something.fcount);

  ignition::physics::CompositeData output;
  something.WriteExpectedData(output);
  EXPECT_EQ(67, output.Get<IntData>().myInt);
  EXPECT_DOUBLE_EQ(7.2, output.Get<DoubleData>().myDouble);
  EXPECT_EQ("seventy-seven", output.Get<StringData>().myString);
  EXPECT_EQ('8', output.Get<CharData>().myChar);
}

/////////////////////////////////////////////////
TEST(CanReadWrite, OnlyReadOnce)
{
  ignition::physics::CompositeData input;
  input.Get<StringData>().myString = "89";
  input.Get<BoolData>().myBool = false;
  input.Get<CharData>().myChar = 'd';
  input.Get<IntData>().myInt = 92;
  input.Get<FloatData>().myFloat = 93.5;
  input.ResetQueries();

  SomeClass<RedundantSpec> redundant;
  redundant.ReadRequiredData(input, ignition::physics::ReadOptions(false));
  redundant.ReadRequiredData(input);
  EXPECT_EQ("89", redundant.sdata.myString);
  EXPECT_FALSE(redundant.bdata.myBool);
  EXPECT_EQ('d', redundant.cdata.myChar);
  EXPECT_EQ(55, redundant.idata.myInt);
  EXPECT_FLOAT_EQ(9.5, redundant.fdata.myFloat);

  redundant.ReadExpectedData(input);
  EXPECT_EQ(92, redundant.idata.myInt);
  EXPECT_FLOAT_EQ(93.5, redundant.fdata.myFloat);

  EXPECT_EQ(1u, redundant.scount);
  EXPECT_EQ(1u, redundant.bcount);
  EXPECT_EQ(1u, redundant.ccount);
  EXPECT_EQ(1u, redundant.icount);
  EXPECT_EQ(1u, redundant.fcount);
}

template <typename ReadSpec>
class SomeClassReadExpected
    : public SomeClassBase<ReadSpec>,
      public CanReadExpectedData<SomeClassReadExpected<ReadSpec>, ReadSpec>
{
  public: SomeClassReadExpected()
    : SomeClassBase<ReadSpec>()
  {
    // Do nothing
  }
};

/////////////////////////////////////////////////
TEST(CanReadWrite, ReadExpected)
{
  SomeClassReadExpected<RequireStringBoolChar> something;
  // expect no reads to have happened yet
  EXPECT_EQ(0u, something.scount);
  EXPECT_EQ(0u, something.bcount);
  EXPECT_EQ(0u, something.ccount);
  EXPECT_EQ(0u, something.icount);
  EXPECT_EQ(0u, something.fcount);

  {
    ignition::physics::CompositeData empty;

    // nothing happens if reading from empty CompositeData
    something.ReadExpectedData(empty);
    EXPECT_EQ(0u, something.scount);
    EXPECT_EQ(0u, something.bcount);
    EXPECT_EQ(0u, something.ccount);
    EXPECT_EQ(0u, something.icount);
    EXPECT_EQ(0u, something.fcount);
  }

  {
    RequireStringBoolChar data;

    // read from data structure that only has its required fields
    // (String, Bool, and Char): only those will be read
    something.ReadExpectedData(data);
    EXPECT_EQ(1u, something.scount);
    EXPECT_EQ(1u, something.bcount);
    EXPECT_EQ(1u, something.ccount);
    EXPECT_EQ(0u, something.icount);
    EXPECT_EQ(0u, something.fcount);

    // read again with default ReadOptions
    // it should skip the required data since it's already been queried
    // so nothing should happen
    something.ReadExpectedData(data);
    EXPECT_EQ(1u, something.scount);
    EXPECT_EQ(1u, something.bcount);
    EXPECT_EQ(1u, something.ccount);
    EXPECT_EQ(0u, something.icount);
    EXPECT_EQ(0u, something.fcount);

    ignition::physics::ReadOptions opt;
    opt.onlyReadUnqueriedData = true;
    // repeat with explicit option to skip queried data
    // again nothing happens
    something.ReadExpectedData(data, opt);
    EXPECT_EQ(1u, something.scount);
    EXPECT_EQ(1u, something.bcount);
    EXPECT_EQ(1u, something.ccount);
    EXPECT_EQ(0u, something.icount);
    EXPECT_EQ(0u, something.fcount);

    opt.onlyReadUnqueriedData = false;
    // now force to read all data
    // data is read again
    something.ReadExpectedData(data, opt);
    EXPECT_EQ(2u, something.scount);
    EXPECT_EQ(2u, something.bcount);
    EXPECT_EQ(2u, something.ccount);
    EXPECT_EQ(0u, something.icount);
    EXPECT_EQ(0u, something.fcount);

    // now add the expected data (Int, Float)
    // and reset queries
    data.Get<IntData>().myInt = 42;
    data.Get<FloatData>().myFloat = 19.99;
    data.ResetQueries();

    // read again with default ReadOptions
    // it should read both Required and Expected data
    something.ReadExpectedData(data);
    EXPECT_EQ(3u, something.scount);
    EXPECT_EQ(3u, something.bcount);
    EXPECT_EQ(3u, something.ccount);
    EXPECT_EQ(1u, something.icount);
    EXPECT_EQ(1u, something.fcount);
  }
}

template <typename ReadSpec>
class SomeClassReadRequired
    : public SomeClassBase<ReadSpec>,
      public CanReadRequiredData<SomeClassReadRequired<ReadSpec>, ReadSpec>
{
  public: SomeClassReadRequired()
    : SomeClassBase<ReadSpec>()
  {
    // Do nothing
  }
};

/////////////////////////////////////////////////
TEST(CanReadWrite, ReadRequired)
{
  SomeClassReadRequired<RequireStringBoolChar> something;
  // expect no reads to have happened yet
  EXPECT_EQ(0u, something.scount);
  EXPECT_EQ(0u, something.bcount);
  EXPECT_EQ(0u, something.ccount);
  EXPECT_EQ(0u, something.icount);
  EXPECT_EQ(0u, something.fcount);

  {
    ignition::physics::CompositeData empty;

    // nothing happens if reading from empty CompositeData
    something.ReadRequiredData(empty);
    EXPECT_EQ(0u, something.scount);
    EXPECT_EQ(0u, something.bcount);
    EXPECT_EQ(0u, something.ccount);
    EXPECT_EQ(0u, something.icount);
    EXPECT_EQ(0u, something.fcount);
  }

  {
    // read from data structure that has both required and expected fields
    RequireStringBoolChar data;
    data.Get<IntData>().myInt = 42;
    data.Get<FloatData>().myFloat = 19.99;

    // (String, Bool, and Char): only those will be read
    something.ReadRequiredData(data);
    EXPECT_EQ(1u, something.scount);
    EXPECT_EQ(1u, something.bcount);
    EXPECT_EQ(1u, something.ccount);
    EXPECT_EQ(0u, something.icount);
    EXPECT_EQ(0u, something.fcount);

    // read again with default ReadOptions
    // it should skip the required data since it's already been queried
    // so nothing should happen
    something.ReadRequiredData(data);
    EXPECT_EQ(1u, something.scount);
    EXPECT_EQ(1u, something.bcount);
    EXPECT_EQ(1u, something.ccount);
    EXPECT_EQ(0u, something.icount);
    EXPECT_EQ(0u, something.fcount);

    ignition::physics::ReadOptions opt;
    opt.onlyReadUnqueriedData = true;
    // repeat with explicit option to skip queried data
    // again nothing happens
    something.ReadRequiredData(data, opt);
    EXPECT_EQ(1u, something.scount);
    EXPECT_EQ(1u, something.bcount);
    EXPECT_EQ(1u, something.ccount);
    EXPECT_EQ(0u, something.icount);
    EXPECT_EQ(0u, something.fcount);

    opt.onlyReadUnqueriedData = false;
    // now force to read all data
    // data is read again
    something.ReadRequiredData(data, opt);
    EXPECT_EQ(2u, something.scount);
    EXPECT_EQ(2u, something.bcount);
    EXPECT_EQ(2u, something.ccount);
    EXPECT_EQ(0u, something.icount);
    EXPECT_EQ(0u, something.fcount);
  }
}

template <typename WriteSpec>
class SomeClassWriteExpected
    : public SomeClassBase<WriteSpec>,
      public CanWriteExpectedData<SomeClassWriteExpected<WriteSpec>, WriteSpec>
{
  public: SomeClassWriteExpected()
    : SomeClassBase<WriteSpec>()
  {
    // Do nothing
  }
};

/////////////////////////////////////////////////
TEST(CanReadWrite, WriteExpected)
{
  SomeClassWriteExpected<RequireIntDouble> something;

  {
    ignition::physics::CompositeData output;
    EXPECT_FALSE(output.Has<DoubleData>());
    EXPECT_FALSE(output.Has<IntData>());
    EXPECT_FALSE(output.Has<StringData>());
    EXPECT_FALSE(output.Has<CharData>());
    EXPECT_EQ(0u, output.AllEntries().size());

    // Write with default options
    something.WriteExpectedData(output);
    EXPECT_TRUE(output.Has<DoubleData>());
    EXPECT_TRUE(output.Has<IntData>());
    EXPECT_TRUE(output.Has<StringData>());
    EXPECT_TRUE(output.Has<CharData>());
    EXPECT_DOUBLE_EQ(7.2, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(67, output.Get<IntData>().myInt);
    EXPECT_EQ("seventy-seven", output.Get<StringData>().myString);
    EXPECT_EQ('8', output.Get<CharData>().myChar);
    EXPECT_EQ(4u, output.AllEntries().size());
  }

  {
    ignition::physics::CompositeData output;
    EXPECT_FALSE(output.Has<DoubleData>());
    EXPECT_FALSE(output.Has<IntData>());
    EXPECT_FALSE(output.Has<StringData>());
    EXPECT_FALSE(output.Has<CharData>());
    EXPECT_EQ(0u, output.AllEntries().size());

    // Write with explicit default options
    ignition::physics::WriteOptions opt;
    opt.skipMissingData = false;
    opt.onlyWriteUnqueriedData = true;
    something.WriteExpectedData(output, opt);
    EXPECT_TRUE(output.Has<DoubleData>());
    EXPECT_TRUE(output.Has<IntData>());
    EXPECT_TRUE(output.Has<StringData>());
    EXPECT_TRUE(output.Has<CharData>());
    EXPECT_DOUBLE_EQ(7.2, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(67, output.Get<IntData>().myInt);
    EXPECT_EQ("seventy-seven", output.Get<StringData>().myString);
    EXPECT_EQ('8', output.Get<CharData>().myChar);
    EXPECT_EQ(4u, output.AllEntries().size());
  }

  {
    ignition::physics::CompositeData output;
    EXPECT_FALSE(output.Has<DoubleData>());
    EXPECT_FALSE(output.Has<IntData>());
    EXPECT_FALSE(output.Has<StringData>());
    EXPECT_FALSE(output.Has<CharData>());
    EXPECT_EQ(0u, output.AllEntries().size());

    // skip missing data
    // output is initially empty, so nothing should be written
    ignition::physics::WriteOptions opt;
    opt.skipMissingData = true;
    opt.onlyWriteUnqueriedData = true;
    something.WriteExpectedData(output, opt);
    EXPECT_FALSE(output.Has<DoubleData>());
    EXPECT_FALSE(output.Has<IntData>());
    EXPECT_FALSE(output.Has<StringData>());
    EXPECT_FALSE(output.Has<CharData>());
    EXPECT_EQ(0u, output.AllEntries().size());

    // create the expected data entries
    output.Get<DoubleData>().myDouble = 1.23;
    output.Get<IntData>().myInt = 123;
    output.Get<StringData>().myString = "initial value";
    output.Get<CharData>().myChar = 'i';
    EXPECT_TRUE(output.Has<DoubleData>());
    EXPECT_TRUE(output.Has<IntData>());
    EXPECT_TRUE(output.Has<StringData>());
    EXPECT_TRUE(output.Has<CharData>());
    EXPECT_DOUBLE_EQ(1.23, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(123, output.Get<IntData>().myInt);
    EXPECT_EQ("initial value", output.Get<StringData>().myString);
    EXPECT_EQ('i', output.Get<CharData>().myChar);
    EXPECT_EQ(4u, output.AllEntries().size());

    // write again with same options
    // but it shouldn't write because data are queried
    something.WriteExpectedData(output, opt);
    EXPECT_DOUBLE_EQ(1.23, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(123, output.Get<IntData>().myInt);
    EXPECT_EQ("initial value", output.Get<StringData>().myString);
    EXPECT_EQ('i', output.Get<CharData>().myChar);

    // unquery and write again
    output.ResetQueries();
    something.WriteExpectedData(output, opt);
    // required data are updated
    EXPECT_DOUBLE_EQ(7.2, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(67, output.Get<IntData>().myInt);
    EXPECT_EQ("seventy-seven", output.Get<StringData>().myString);
    EXPECT_EQ('8', output.Get<CharData>().myChar);

    // change value, which marks as queried
    output.Get<DoubleData>().myDouble = 4.56;
    output.Get<IntData>().myInt = 456;
    output.Get<StringData>().myString = "queried again";
    output.Get<CharData>().myChar = 'Q';

    // write again with same options, and it won't change
    something.WriteExpectedData(output, opt);
    EXPECT_DOUBLE_EQ(4.56, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(456, output.Get<IntData>().myInt);
    EXPECT_EQ("queried again", output.Get<StringData>().myString);
    EXPECT_EQ('Q', output.Get<CharData>().myChar);

    // change options to write all data regardless of query status
    // write and expect change
    opt.onlyWriteUnqueriedData = false;
    something.WriteExpectedData(output, opt);
    EXPECT_DOUBLE_EQ(7.2, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(67, output.Get<IntData>().myInt);
    EXPECT_EQ("seventy-seven", output.Get<StringData>().myString);
    EXPECT_EQ('8', output.Get<CharData>().myChar);
  }
}

template <typename WriteSpec>
class SomeClassWriteRequired
    : public SomeClassBase<WriteSpec>,
      public CanWriteRequiredData<SomeClassWriteRequired<WriteSpec>, WriteSpec>
{
  public: SomeClassWriteRequired()
    : SomeClassBase<WriteSpec>()
  {
    // Do nothing
  }
};

/////////////////////////////////////////////////
TEST(CanReadWrite, WriteRequired)
{
  SomeClassWriteRequired<RequireIntDouble> something;

  {
    ignition::physics::CompositeData output;
    EXPECT_FALSE(output.Has<DoubleData>());
    EXPECT_FALSE(output.Has<IntData>());
    EXPECT_EQ(0u, output.AllEntries().size());

    // Write with default options
    something.WriteRequiredData(output);
    EXPECT_TRUE(output.Has<DoubleData>());
    EXPECT_TRUE(output.Has<IntData>());
    EXPECT_DOUBLE_EQ(7.2, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(67, output.Get<IntData>().myInt);
    EXPECT_EQ(2u, output.AllEntries().size());
  }

  {
    ignition::physics::CompositeData output;
    EXPECT_FALSE(output.Has<DoubleData>());
    EXPECT_FALSE(output.Has<IntData>());
    EXPECT_EQ(0u, output.AllEntries().size());

    // Write with explicit default options
    ignition::physics::WriteOptions opt;
    opt.skipMissingData = false;
    opt.onlyWriteUnqueriedData = true;
    something.WriteRequiredData(output, opt);
    EXPECT_TRUE(output.Has<DoubleData>());
    EXPECT_TRUE(output.Has<IntData>());
    EXPECT_DOUBLE_EQ(7.2, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(67, output.Get<IntData>().myInt);
    EXPECT_EQ(2u, output.AllEntries().size());
  }

  {
    ignition::physics::CompositeData output;
    EXPECT_FALSE(output.Has<DoubleData>());
    EXPECT_FALSE(output.Has<IntData>());
    EXPECT_EQ(0u, output.AllEntries().size());

    // skip missing data
    // output is initially empty, so nothing should be written
    ignition::physics::WriteOptions opt;
    opt.skipMissingData = true;
    opt.onlyWriteUnqueriedData = true;
    something.WriteRequiredData(output, opt);
    EXPECT_FALSE(output.Has<DoubleData>());
    EXPECT_FALSE(output.Has<IntData>());
    EXPECT_EQ(0u, output.AllEntries().size());

    // create the required data entries and an expected one too
    output.Get<DoubleData>().myDouble = 1.23;
    output.Get<IntData>().myInt = 123;
    output.Get<StringData>().myString = "initial value";
    EXPECT_TRUE(output.Has<DoubleData>());
    EXPECT_TRUE(output.Has<IntData>());
    EXPECT_TRUE(output.Has<StringData>());
    EXPECT_DOUBLE_EQ(1.23, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(123, output.Get<IntData>().myInt);
    EXPECT_EQ("initial value", output.Get<StringData>().myString);
    EXPECT_EQ(3u, output.AllEntries().size());

    // write again with same options
    // but it shouldn't write because data are queried
    something.WriteRequiredData(output, opt);
    EXPECT_DOUBLE_EQ(1.23, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(123, output.Get<IntData>().myInt);
    EXPECT_EQ("initial value", output.Get<StringData>().myString);

    // unquery and write again
    output.ResetQueries();
    something.WriteRequiredData(output, opt);
    // required data are updated
    EXPECT_DOUBLE_EQ(7.2, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(67, output.Get<IntData>().myInt);
    // expected data is not changed
    EXPECT_EQ("initial value", output.Get<StringData>().myString);

    // change value, which marks as queried
    output.Get<DoubleData>().myDouble = 4.56;
    output.Get<IntData>().myInt = 456;
    output.Get<StringData>().myString = "queried again";

    // write again with same options, and it won't change
    something.WriteRequiredData(output, opt);
    EXPECT_DOUBLE_EQ(4.56, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(456, output.Get<IntData>().myInt);
    EXPECT_EQ("queried again", output.Get<StringData>().myString);

    // change options to write all data regardless of query status
    // write and expect change
    opt.onlyWriteUnqueriedData = false;
    something.WriteRequiredData(output, opt);
    EXPECT_DOUBLE_EQ(7.2, output.Get<DoubleData>().myDouble);
    EXPECT_EQ(67, output.Get<IntData>().myInt);
    // expected data is not changed
    EXPECT_EQ("queried again", output.Get<StringData>().myString);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
