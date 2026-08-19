/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <gz/plugin/Loader.hh>
#include "mock/MockCompositeData.hh"

TEST(Issue1019Test, TypeIndexAcrossDSO)
{
  gz::plugin::Loader loader;
  const auto pluginNames = loader.LoadLib(MockCompositeData_LIB);
  ASSERT_EQ(1u, pluginNames.size());

  auto plugin = loader.Instantiate("mock::CompositeDataPlugin");
  ASSERT_TRUE(plugin);

  auto *mockPlugin = plugin->QueryInterface<mock::MockCompositeDataPlugin>();
  ASSERT_NE(nullptr, mockPlugin);

  gz::physics::CompositeData data = mockPlugin->GetCompositeData();

  // In PR #1019, CompositeData was changed to use std::type_index as the map
  // key. When a plugin DSO is compiled with hidden visibility for
  // inlines/templates (e.g. in pixi / conda environments),
  // std::type_index(typeid(ExtraContactData)) has a different pointer in the
  // test binary than in the plugin DSO, causing Query to fail (return nullptr).
  const auto *extra = data.Query<mock::ExtraContactData>();
  ASSERT_NE(nullptr, extra);
  EXPECT_DOUBLE_EQ(42.0, extra->depth);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
