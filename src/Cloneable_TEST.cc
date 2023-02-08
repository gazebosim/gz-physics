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

#include "gz/physics/Cloneable.hh"
#include "TestDataTypes.hh"

using namespace gz;
using physics::Cloneable;
using physics::MakeCloneable;

/////////////////////////////////////////////////
TEST(Cloneable_TEST, Construct)
{
  {
    // Test the default constructor
    MakeCloneable<StringData> stringData;
    EXPECT_EQ("default", stringData.myString);
  }

  {
    // Test single-argument constructors using parentheses
    MakeCloneable<StringData> stringData("some string");
    EXPECT_EQ("some string", stringData.myString);

    MakeCloneable<StringData> copyConstructor(stringData);
    EXPECT_EQ("some string", copyConstructor.myString);

    MakeCloneable<StringData> moveConstructor(std::move(stringData));
    EXPECT_EQ("some string", moveConstructor.myString);
  }

  {
    // Test single-argument constructors using curly braces
    MakeCloneable<StringData> stringData{"another string"};
    EXPECT_EQ("another string", stringData.myString);

    MakeCloneable<StringData> copyConstructor{stringData};
    EXPECT_EQ("another string", copyConstructor.myString);

    MakeCloneable<StringData> moveConstructor{std::move(stringData)};
    EXPECT_EQ("another string", moveConstructor.myString);
  }

  {
    // Test multi-argument constructors using parentheses
    MakeCloneable<MultiData> multiData("using parens", 44);
    EXPECT_EQ("using parens", multiData.myString);
    EXPECT_EQ(44, multiData.myInt);

    MakeCloneable<MultiData> copyConstructor(multiData);
    EXPECT_EQ("using parens", copyConstructor.myString);
    EXPECT_EQ(44, copyConstructor.myInt);

    MakeCloneable<MultiData> moveConstructor(multiData);
    EXPECT_EQ("using parens", moveConstructor.myString);
    EXPECT_EQ(44, moveConstructor.myInt);
  }

  {
    // Test multi-argument constructors using curly braces
    MakeCloneable<MultiData> multiData{"using curl braces", 50};
    EXPECT_EQ("using curl braces", multiData.myString);
    EXPECT_EQ(50, multiData.myInt);

    MakeCloneable<MultiData> copyConstructor{multiData};
    EXPECT_EQ("using curl braces", copyConstructor.myString);
    EXPECT_EQ(50, copyConstructor.myInt);

    MakeCloneable<MultiData> moveConstructor{std::move(multiData)};
    EXPECT_EQ("using curl braces", moveConstructor.myString);
    EXPECT_EQ(50, moveConstructor.myInt);
  }
}

/////////////////////////////////////////////////
TEST(Cloneable_TEST, Assign)
{
  MakeCloneable<StringData> stringData;
  EXPECT_EQ("default", stringData.myString);

  const char *lvalueCharLiteral = "lvalue-CharLiteral";
  stringData = lvalueCharLiteral;
  EXPECT_EQ("lvalue-CharLiteral", stringData.myString);

  stringData = "rvalue-CharLiteral";
  EXPECT_EQ("rvalue-CharLiteral", stringData.myString);

  const std::string lvalueString("lvalue-std::string");
  stringData = lvalueString;
  EXPECT_EQ("lvalue-std::string", stringData.myString);

  stringData = std::string("rvalue-std::string");
  EXPECT_EQ("rvalue-std::string", stringData.myString);

  StringData lvalueStringData("lvalue-StringData");
  stringData = lvalueStringData;
  EXPECT_EQ("lvalue-StringData", stringData.myString);

  stringData = StringData("rvalue-StringData");
  EXPECT_EQ("rvalue-StringData", stringData.myString);

  MakeCloneable<StringData> lvalueMakeCloneable("lvalue-MakeCloneable");
  stringData = lvalueMakeCloneable;
  EXPECT_EQ("lvalue-MakeCloneable", stringData.myString);

  stringData = MakeCloneable<StringData>("rvalue-MakeCloneable");
  EXPECT_EQ("rvalue-MakeCloneable", stringData.myString);
}

/////////////////////////////////////////////////
TEST(Cloneable_TEST, Clone)
{
  MakeCloneable<StringData> stringData("this will be cloned");
  std::unique_ptr<Cloneable> cloned = stringData.Clone();
  std::unique_ptr<Cloneable> moreCloned = cloned->Clone();

  MakeCloneable<StringData> *clonedCasted =
      dynamic_cast<MakeCloneable<StringData>*>(cloned.get());
  ASSERT_NE(nullptr, clonedCasted);
  EXPECT_EQ("this will be cloned", clonedCasted->myString);

  MakeCloneable<StringData> *moreClonedCasted =
      dynamic_cast<MakeCloneable<StringData>*>(moreCloned.get());
  ASSERT_NE(nullptr, moreClonedCasted);
  EXPECT_EQ("this will be cloned", moreClonedCasted->myString);
}

/////////////////////////////////////////////////
TEST(Cloneable_TEST, Copy)
{
  std::unique_ptr<Cloneable> copyFrom =
      std::make_unique<MakeCloneable<StringData>>("copyingFrom");
  MakeCloneable<StringData> *copyFromCasted =
      dynamic_cast<MakeCloneable<StringData>*>(copyFrom.get());
  ASSERT_NE(nullptr, copyFromCasted);
  EXPECT_EQ("copyingFrom", copyFromCasted->myString);

  std::unique_ptr<Cloneable> moveFrom =
      std::make_unique<MakeCloneable<StringData>>("movingFrom");
  MakeCloneable<StringData> *moveFromCasted =
      dynamic_cast<MakeCloneable<StringData>*>(moveFrom.get());
  ASSERT_NE(nullptr, moveFromCasted);
  EXPECT_EQ("movingFrom", moveFromCasted->myString);

  std::unique_ptr<Cloneable> copyTo =
      std::make_unique<MakeCloneable<StringData>>("copyingTo");
  MakeCloneable<StringData> *copyToCasted =
      dynamic_cast<MakeCloneable<StringData>*>(copyTo.get());
  ASSERT_NE(nullptr, copyToCasted);
  EXPECT_EQ("copyingTo", copyToCasted->myString);


  copyTo->Copy(*copyFrom);
  EXPECT_EQ("copyingFrom", copyToCasted->myString);

  copyTo->Copy(std::move(*moveFrom));
  EXPECT_EQ("movingFrom", copyToCasted->myString);
}
