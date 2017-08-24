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


class SomeClass
    : public ignition::physics::CanReadRequiredData<SomeClass, RequireStringBoolChar>,
      public ignition::physics::CanReadExpectedData<SomeClass, RequireStringBoolChar>,
      public ignition::physics::CanWriteExpectedData<SomeClass, RequireIntDouble>
{

  public: void Read(const StringData& sdata)
  {
    std::cout << "Successfully read string: " << sdata.myString << std::endl;
  }

  public: void Read(const BoolData& bdata)
  {
    std::cout << "Sucessfully read bool: " << bdata.myBool << std::endl;
  }

  public: void Read(const CharData& cdata)
  {
    std::cout << "Successfully read char: " << cdata.myChar << std::endl;
  }

  public: void Read(const IntData& idata)
  {
    std::cout << "Successfully read int: " << idata.myInt << std::endl;
  }

  public: void Read(const FloatData& fdata)
  {
    std::cout << "Successfully read float: " << fdata.myFloat << std::endl;
  }

  public: void Write(IntData& idata)
  {

  }

  public: void Write(DoubleData& ddata)
  {

  }

  public: void Write(StringData& sdata)
  {

  }

  public: void Write(CharData& cdata)
  {

  }
};

TEST(SpecifyData, ReadData)
{
  SomeClass some;
  some.ReadRequiredData(RequireStringBoolChar{});
  some.ReadExpectedData(RequireStringBoolChar{});
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
