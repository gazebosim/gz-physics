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

#ifndef IGNITION_PHYSICS_TESTUTILS_HH_
#define IGNITION_PHYSICS_TESTUTILS_HH_

#include "ignition/physics/CompositeData.hh"
#include "ignition/physics/CompositeDataMacros.hh"

/////////////////////////////////////////////////
class StringData
{
  IGN_PHYSICS_DATA_LABEL(StringData)
  public: std::string myString;

  inline StringData(const std::string &_input = "default")
    : myString(_input)
  {
    // Do nothing
  }
};

/////////////////////////////////////////////////
class DoubleData
{
  IGN_PHYSICS_DATA_LABEL(DoubleData)
  public: double myDouble;

  inline DoubleData(const double _input = 1.61803)
    : myDouble(_input)
  {
    // Do nothing
  }
};

/////////////////////////////////////////////////
class IntData
{
  IGN_PHYSICS_DATA_LABEL(IntData)
  public: int myInt;

  inline IntData(const int _input = 55)
    : myInt(_input)
  {
    // Do nothing
  }
};

/////////////////////////////////////////////////
class BoolData
{
  IGN_PHYSICS_DATA_LABEL(BoolData)
  public: bool myBool;

  inline BoolData(const bool _input = true)
    : myBool(_input)
  {
    // Do nothing
  }
};

/////////////////////////////////////////////////
class CharData
{
  IGN_PHYSICS_DATA_LABEL(CharData)
  public: char myChar;

  inline CharData(const char _input = 'c')
    : myChar(_input)
  {
    // Do nothing
  }
};

/////////////////////////////////////////////////
template <typename... DataTypes>
struct AddSomeData
{
  // This class definition is just here for syntax reasons
};

template <typename DataType>
struct AddSomeData<DataType>
{
  static void To(ignition::physics::CompositeData &data)
  {
    data.Create<DataType>();
  }
};

template <typename DataType1, typename... OtherDataTypes>
struct AddSomeData<DataType1, OtherDataTypes...>
{
  static void To(ignition::physics::CompositeData &data)
  {
    data.Create<DataType1>();
    AddSomeData<OtherDataTypes...>::To(data);
  }
};

/////////////////////////////////////////////////
template <typename... DataTypes>
ignition::physics::CompositeData CreateSomeData(bool resetQueries = false)
{
  ignition::physics::CompositeData data;
  AddSomeData<DataTypes...>::To(data);

  if(resetQueries)
    data.ResetQueries();

  return data;
}


#endif
