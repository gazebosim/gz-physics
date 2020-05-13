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

#include <ignition/physics/FeatureList.hh>

class ClassA : public ignition::physics::Feature { };
class ClassB : public ignition::physics::Feature { };
class ClassC : public ignition::physics::Feature { };
class ClassD : public ClassB { };

using Filter = std::tuple<ClassA, ClassD>;
using Input = std::tuple<ClassB, ClassC, ClassD, ClassA>;

using Result = ignition::physics::detail::SubtractTuple<Filter>
    ::From<Input>::type;

using UnfilteredResult =
    ignition::physics::detail::SubtractTuple<std::tuple<>>::From<Input>::type;

TEST(FilterTuple_TEST, FilterTupleResult)
{
  // ClassA and ClassD should be filtered out because they are in Filter.
  // ClassB should be filtered out because it is a base class of ClassD.
  // Therefore we should only be left with ClassC.
  EXPECT_EQ(1u, std::tuple_size<Result>::value);

  EXPECT_EQ(4u, std::tuple_size<UnfilteredResult>::value);
  std::cout << typeid(UnfilteredResult).name() << std::endl;
}


using SingleCombineLists =
  ignition::physics::detail::CombineListsImpl<
      std::tuple<>, ClassA>::type;

using SingleCombineListsInitial =
  ignition::physics::detail::CombineListsImpl<
      std::tuple<>, ClassA>::InitialResult;

using SingleCombineListsPartial =
  ignition::physics::detail::CombineListsImpl<
      std::tuple<>, ClassA>::PartialResult;

using SingleCombineListsChildFilter =
  ignition::physics::detail::CombineListsImpl<
      std::tuple<>, ClassA>::ChildFilter;

TEST(FilterTuple_TEST, CombineListsResult)
{
  EXPECT_EQ(1u, std::tuple_size<SingleCombineListsInitial>::value);
  EXPECT_EQ(1u, std::tuple_size<SingleCombineListsPartial>::value);
  EXPECT_EQ(1u, std::tuple_size<SingleCombineListsChildFilter>::value);
  EXPECT_EQ(1u, std::tuple_size<SingleCombineLists>::value);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
