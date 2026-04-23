/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
 */

#include <gtest/gtest.h>

#include <memory>
#include <type_traits>

#include "Base.hh"

using gz::physics::mujoco::WorldInfo;

// WorldInfo owns exclusive MuJoCo resources — copies must be impossible.
static_assert(!std::is_copy_constructible<WorldInfo>::value,
    "WorldInfo owns MuJoCo resources; copy construction causes double-free");
static_assert(!std::is_copy_assignable<WorldInfo>::value,
    "WorldInfo owns MuJoCo resources; copy assignment causes double-free");

/////////////////////////////////////////////////
// Destructor with all-null pointers must not crash.
// mj_delete* functions are null-safe; this test pins that guarantee.
TEST(WorldInfoDestructor, NullPointersNoCrash)
{
  auto info = std::make_shared<WorldInfo>();
  EXPECT_EQ(nullptr, info->mjSpecObj);
  EXPECT_EQ(nullptr, info->mjModelObj);
  EXPECT_EQ(nullptr, info->mjDataObj);
  EXPECT_NO_FATAL_FAILURE(info.reset());
}

/////////////////////////////////////////////////
// Destructor must release live MuJoCo objects without crash.
// Under ASan/LSan a leak here is reported as "definitely lost".
TEST(WorldInfoDestructor, LiveObjectsReleasedOnDestruction)
{
  {
    auto info = std::make_shared<WorldInfo>();

    info->mjSpecObj = mj_makeSpec();
    ASSERT_NE(nullptr, info->mjSpecObj);

    info->mjModelObj = mj_compile(info->mjSpecObj, nullptr);
    ASSERT_NE(nullptr, info->mjModelObj);

    info->mjDataObj = mj_makeData(info->mjModelObj);
    ASSERT_NE(nullptr, info->mjDataObj);
  }
  SUCCEED();
}

/////////////////////////////////////////////////
// Repeated construction/destruction cycles must not accumulate leaks.
// If the destructor silently fails, repeated iterations exhaust memory.
TEST(WorldInfoDestructor, RepeatedCycleNoLeakAccumulation)
{
  for (int i = 0; i < 10; ++i)
  {
    auto info = std::make_shared<WorldInfo>();

    info->mjSpecObj = mj_makeSpec();
    ASSERT_NE(nullptr, info->mjSpecObj) << "iteration " << i;

    info->mjModelObj = mj_compile(info->mjSpecObj, nullptr);
    ASSERT_NE(nullptr, info->mjModelObj) << "iteration " << i;

    info->mjDataObj = mj_makeData(info->mjModelObj);
    ASSERT_NE(nullptr, info->mjDataObj) << "iteration " << i;

    EXPECT_NO_FATAL_FAILURE(info.reset()) << "iteration " << i;
  }
}
