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

#include <ignition/physics/RequestEngine.hh>

using namespace ignition::physics;

class FeatureA : public virtual Feature { };
class FeatureB : public virtual Feature { };
class FeatureC : public virtual Feature { };

class Conflict1 : public virtual Feature { };
class Conflict2 : public virtual Feature { };
class Conflict3 : public virtual Feature { };

class OnlyConflictWith1
    : public virtual FeatureWithConflicts<Conflict1> { };

class MultipleConflicts
    : public virtual FeatureWithConflicts<Conflict1, Conflict2, Conflict3> { };

#define CREATE_FEATURE_LISTS(x) \
  using x ## List1 = FeatureList<x, FeatureA, FeatureB, FeatureC>; \
  using x ## List2 = FeatureList<FeatureA, x, FeatureB, FeatureC>; \
  using x ## List3 = FeatureList<FeatureA, FeatureB, x, FeatureC>; \
  using x ## List4 = FeatureList<FeatureA, FeatureB, FeatureC, x>;

CREATE_FEATURE_LISTS(Conflict1)
CREATE_FEATURE_LISTS(Conflict2)
CREATE_FEATURE_LISTS(Conflict3)
CREATE_FEATURE_LISTS(FeatureA)
CREATE_FEATURE_LISTS(OnlyConflictWith1)
CREATE_FEATURE_LISTS(MultipleConflicts)


#define FEATURE_AND_ITS_LISTS(x) \
  x , x ## List1, x ## List2, x ## List3, x ## List4

template <bool _expectConflicts, int _line, typename LHS>
void ExpectConflicts()
{
  // Do nothing. This is where the variadic template function terminates.
}

template <bool _expectConflicts, int _line, typename LHS, typename RHS,
          typename... MoreRHS>
void ExpectConflicts()
{
  const bool conflict = LHS::template ConflictsWith<RHS>();

  if (_expectConflicts)
    EXPECT_TRUE(conflict) << " originating from line " << _line;
  else
    EXPECT_FALSE(conflict) << " originating from line " << _line;

  ExpectConflicts<_expectConflicts, _line, LHS, MoreRHS...>();
}

/// \brief This class allows us to specify a list of left-hand side types to
/// check for conflicts with a list of right-hand side types. The RHS types get
/// passed to the static member function With<...>().
///
/// _expectConflicts should be set to true when conflicts are expected, and
/// false when they are not expected.
template <bool _expectConflicts, int _line, typename... MoreLHS>
struct TestConflicts
{
  public: template <typename... RHS>
  static void With()
  {
    // Do nothing. This is where the recursive template class terminates.
  }
};

/// \brief This is a specialization of TestConflicts which actually does the
/// heavy lifting.
template <bool _expectConflicts, int _line, typename LHS1, typename... MoreLHS>
struct TestConflicts<_expectConflicts, _line, LHS1, MoreLHS...>
{
  public: template <typename... RHS>
  static void With()
  {
    ExpectConflicts<_expectConflicts, _line, LHS1, RHS...>();
    TestConflicts<_expectConflicts, _line, MoreLHS...>::template With<RHS...>();
  }
};

TEST(FeatureList_TEST, Conflicts)
{
  // Note: This test might be somewhat difficult to read, but we have
  // effectively collapsed 600+ EXPECT_XXX calls into the following ~35 lines.
  //
  // Here's a breakdown of what's happening:
  // TestConflicts<bool, ...LHS...>::With<...RHS...>() tests each type listed
  // in LHS (the left-hand side) for conflicts with each type listed in RHS
  // (the right-hand side). If bool is true, we expect each test to conflict. If
  // bool is false, we expect each test to not conflict.
  //
  // FEATURE_AND_ITS_LISTS(X) expands to a comma-separated sequence that
  // includes the feature X and a set of FeatureLists which contain X. We use
  // this macro because any conflict properties of a feature should also be
  // present in any FeatureList that the feature is added to.

  TestConflicts<true, __LINE__,
      FEATURE_AND_ITS_LISTS(OnlyConflictWith1)>
        ::With<
          FEATURE_AND_ITS_LISTS(Conflict1)>();

  EXPECT_TRUE(FeatureList<Conflict1>::ConflictsWith<OnlyConflictWith1>());

  // Note: OnlyConflictWith1 only conflicts with Conflict1. It does not conflict
  // with Conflict2 or Conflict3.
  TestConflicts<false, __LINE__,
      FEATURE_AND_ITS_LISTS(OnlyConflictWith1)>
        ::With<
          FEATURE_AND_ITS_LISTS(Conflict2),
          FEATURE_AND_ITS_LISTS(Conflict3),
          FEATURE_AND_ITS_LISTS(FeatureA),
          FEATURE_AND_ITS_LISTS(MultipleConflicts)>();

  TestConflicts<true, __LINE__,
      FEATURE_AND_ITS_LISTS(MultipleConflicts)>
        ::With<
          FEATURE_AND_ITS_LISTS(Conflict1),
          FEATURE_AND_ITS_LISTS(Conflict2),
          FEATURE_AND_ITS_LISTS(Conflict3)>();

  TestConflicts<false, __LINE__,
      FEATURE_AND_ITS_LISTS(MultipleConflicts)>
        ::With<
          FEATURE_AND_ITS_LISTS(FeatureA)>();

  TestConflicts<false, __LINE__,
      FEATURE_AND_ITS_LISTS(FeatureA),
      FEATURE_AND_ITS_LISTS(Conflict1),
      FEATURE_AND_ITS_LISTS(Conflict2),
      FEATURE_AND_ITS_LISTS(Conflict3)>
        ::With<
          FEATURE_AND_ITS_LISTS(FeatureA),
          FEATURE_AND_ITS_LISTS(Conflict1),
          FEATURE_AND_ITS_LISTS(Conflict2),
          FEATURE_AND_ITS_LISTS(Conflict3)>();
}

class RequiresFeatureA
    : public virtual FeatureWithRequirements<FeatureA> { };

class RequiresFeaturesBC
    : public virtual FeatureWithRequirements<FeatureB, FeatureC> { };

TEST(FeatureList_TEST, Requirements)
{
  // These tests are making sure that required features are getting added to the
  // FeatureList where they're needed.

  using List1 = FeatureList<RequiresFeatureA>;
  EXPECT_TRUE(List1::HasFeature<FeatureA>());
  EXPECT_FALSE(List1::HasFeature<FeatureB>());
  EXPECT_FALSE(List1::HasFeature<FeatureC>());

  using List2 = FeatureList<RequiresFeaturesBC>;
  EXPECT_FALSE(List2::HasFeature<FeatureA>());
  EXPECT_TRUE(List2::HasFeature<FeatureB>());
  EXPECT_TRUE(List2::HasFeature<FeatureC>());

  using List3 = FeatureList<RequiresFeatureA,
                            RequiresFeaturesBC>;
  EXPECT_TRUE(List3::HasFeature<FeatureA>());
  EXPECT_TRUE(List3::HasFeature<FeatureB>());
  EXPECT_TRUE(List3::HasFeature<FeatureC>());

  using List4 = FeatureList<
      Conflict1, Conflict2, Conflict3,
      RequiresFeatureA,
      RequiresFeaturesBC>;
  EXPECT_TRUE(List4::HasFeature<FeatureA>());
  EXPECT_TRUE(List4::HasFeature<FeatureB>());
  EXPECT_TRUE(List4::HasFeature<FeatureC>());

  using List5 = FeatureList<
      RequiresFeatureA,
      RequiresFeaturesBC,
      Conflict1, Conflict2, Conflict3>;
  EXPECT_TRUE(List5::HasFeature<FeatureA>());
  EXPECT_TRUE(List5::HasFeature<FeatureB>());
  EXPECT_TRUE(List5::HasFeature<FeatureC>());

  using List6 = FeatureList<Conflict1, Conflict2, Conflict3, List1>;
  EXPECT_TRUE(List6::HasFeature<FeatureA>());
  EXPECT_FALSE(List6::HasFeature<FeatureB>());
  EXPECT_FALSE(List6::HasFeature<FeatureC>());
}

TEST(FeatureList_TEST, ConflictsAndRequirements)
{
  // Test against an infinitely recursive template instantiation issue.
  // We need to define a class unique to this test, otherwise the regression
  // test might not catch the issue if a FeatureList with TestRequiresFeatureA
  // gets instantiated beforehand.
  class TestRequiresFeatureA
      : public virtual FeatureWithRequirements<FeatureA> { };

  // This is a regression test. We need to make sure that this FeatureList
  // compiles.
  ignition::physics::FeatureList<OnlyConflictWith1, TestRequiresFeatureA>();
}

TEST(FeatureList_TEST, Hierarchy)
{
  using HierarchyLevel1 = FeatureList<FeatureA, FeatureB>;
  using HierarchyLevel2a = FeatureList<HierarchyLevel1, FeatureC>;
  using HierarchyLevel2b = FeatureList<Conflict1, Conflict2>;
  using HierarchyLevel3 = FeatureList<HierarchyLevel2a, HierarchyLevel2b>;

  // This test makes sure that a hierarchy of FeatureLists can compile.
  // As long as the line below can compile, the test is passed.
  HierarchyLevel3();

  using Level3Tuple = detail::FlattenFeatures<HierarchyLevel3>::type;
  EXPECT_TRUE((detail::TupleContainsBase<FeatureA, Level3Tuple>::value));
  EXPECT_TRUE((detail::TupleContainsBase<FeatureB, Level3Tuple>::value));
  EXPECT_TRUE((detail::TupleContainsBase<FeatureC, Level3Tuple>::value));
  EXPECT_TRUE((detail::TupleContainsBase<Conflict1, Level3Tuple>::value));
  EXPECT_TRUE((detail::TupleContainsBase<Conflict2, Level3Tuple>::value));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
