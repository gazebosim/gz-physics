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

#include <ignition/physics/Update.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/ForwardStep.hh>

#include <gtest/gtest.h>

using namespace ignition::physics;

class Conflict1 : public virtual Feature { };
class Conflict2 : public virtual Feature { };
class Conflict3 : public virtual Feature { };

class OnlyConflictWith1
    : public virtual FeatureWithConflicts<Conflict1> { };

class MultipleConflicts
    : public virtual FeatureWithConflicts<Conflict1, Conflict2, Conflict3> { };

class NoConflicts : public virtual Feature { };

#define CREATE_FEATURE_LISTS(x) \
  using x ## List1 = FeatureList<x, FrameSemantics, ForwardStep, SetState>; \
  using x ## List2 = FeatureList<FrameSemantics, x, ForwardStep, SetState>; \
  using x ## List3 = FeatureList<FrameSemantics, ForwardStep, x, SetState>; \
  using x ## List4 = FeatureList<FrameSemantics, ForwardStep, SetState, x>;

CREATE_FEATURE_LISTS(Conflict1)
CREATE_FEATURE_LISTS(Conflict2)
CREATE_FEATURE_LISTS(Conflict3)
CREATE_FEATURE_LISTS(NoConflicts)
CREATE_FEATURE_LISTS(OnlyConflictWith1)
CREATE_FEATURE_LISTS(MultipleConflicts)


#define FEATURE_AND_ITS_LISTS(x) \
  x , x ## List1, x ## List2, x ## List3, x ## List4

template <bool _expectConflicts, typename LHS>
void ExpectConflicts()
{
  // Do nothing. This is where the variadic template function terminates.
}

template <bool _expectConflicts, typename LHS, typename RHS, typename... MoreRHS>
void ExpectConflicts()
{
  if (_expectConflicts)
    EXPECT_TRUE(LHS::template ConflictsWith<RHS>());
  else
    EXPECT_FALSE(LHS::template ConflictsWith<RHS>());

  ExpectConflicts<_expectConflicts, LHS, MoreRHS...>();
}

/// \brief This class allows us to specify a list of left-hand side types to
/// check for conflicts with a list of right-hand side types. The RHS types get
/// passed to the static member function With<...>().
///
/// _expectConflicts should be set to true when conflicts are expected, and
/// false when they are not expected.
template <bool _expectConflicts, typename... MoreLHS>
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
template <bool _expectConflicts, typename LHS1, typename... MoreLHS>
struct TestConflicts<_expectConflicts, LHS1, MoreLHS...>
{
  public: template <typename... RHS>
  static void With()
  {
    ExpectConflicts<_expectConflicts, LHS1, RHS...>();
    TestConflicts<_expectConflicts, MoreLHS...>::template With<RHS...>();
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

  TestConflicts<true,
      FEATURE_AND_ITS_LISTS(OnlyConflictWith1)>
        ::With<
          FEATURE_AND_ITS_LISTS(Conflict1)>();

  // Note: OnlyConflictWith1 only conflicts with Conflict1. It does not conflict
  // with Conflict2 or Conflict3.
  TestConflicts<false,
      FEATURE_AND_ITS_LISTS(OnlyConflictWith1)>
        ::With<
          FEATURE_AND_ITS_LISTS(Conflict2),
          FEATURE_AND_ITS_LISTS(Conflict3),
          FEATURE_AND_ITS_LISTS(NoConflicts),
          FEATURE_AND_ITS_LISTS(MultipleConflicts)>();

  TestConflicts<true,
      FEATURE_AND_ITS_LISTS(MultipleConflicts)>
        ::With<
          FEATURE_AND_ITS_LISTS(Conflict1),
          FEATURE_AND_ITS_LISTS(Conflict2),
          FEATURE_AND_ITS_LISTS(Conflict3)>();

  TestConflicts<false,
      FEATURE_AND_ITS_LISTS(MultipleConflicts)>
        ::With<
          FEATURE_AND_ITS_LISTS(NoConflicts)>();

  TestConflicts<false,
      FEATURE_AND_ITS_LISTS(NoConflicts),
      FEATURE_AND_ITS_LISTS(Conflict1),
      FEATURE_AND_ITS_LISTS(Conflict2),
      FEATURE_AND_ITS_LISTS(Conflict3)>
        ::With<
          FEATURE_AND_ITS_LISTS(NoConflicts),
          FEATURE_AND_ITS_LISTS(Conflict1),
          FEATURE_AND_ITS_LISTS(Conflict2),
          FEATURE_AND_ITS_LISTS(Conflict3)>();

  struct FT
  {
    using Scalar = double;
    enum { Dim = 3 };
  };

//  using ShouldNotCompile = FeatureList<Conflict1List2>;
//  ShouldNotCompile::Link<FT> link;
////  Conflict1List2::Link<FT> link;
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
