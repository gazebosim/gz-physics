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

#include <gz/physics/FeatureList.hh>

using namespace gz;
class ClassA : public physics::Feature { };
class ClassB : public physics::Feature { };
class ClassC : public physics::Feature { };
class ClassD : public ClassB { };

using Filter = physics::TypeList<ClassA, ClassD>;
using Input = physics::TypeList<ClassB, ClassC, ClassD, ClassA>;

using Result = physics::detail::SubtractList<Filter>
    ::From<Input>::type;

using UnfilteredResult =
    physics::detail::SubtractList<physics::TypeList<>>::From<Input>::type;

TEST(FilterTypeList_TEST, FilterTypeListResult)
{
  // ClassA and ClassD should be filtered out because they are in Filter.
  // ClassB should be filtered out because it is a base class of ClassD.
  // Therefore we should only be left with ClassC.
  EXPECT_EQ(1u, Result::size);

  EXPECT_EQ(4u, UnfilteredResult::size);
  std::cout << typeid(UnfilteredResult).name() << std::endl;
}


using SingleCombineLists =
  physics::detail::CombineListsImpl<
      physics::TypeList<>, ClassA>::type;

using SingleCombineListsInitial =
  physics::detail::CombineListsImpl<
      physics::TypeList<>, ClassA>::InitialResult;

using SingleCombineListsPartial =
  physics::detail::CombineListsImpl<
      physics::TypeList<>, ClassA>::PartialResult;

using SingleCombineListsChildFilter =
  physics::detail::CombineListsImpl<
      physics::TypeList<>, ClassA>::ChildFilter;

TEST(FilterTypeList_TEST, CombineListsResult)
{
  EXPECT_EQ(1u, SingleCombineListsInitial::size);
  EXPECT_EQ(1u, SingleCombineListsPartial::size);
  EXPECT_EQ(1u, SingleCombineListsChildFilter::size);
  EXPECT_EQ(1u, SingleCombineLists::size);
}
