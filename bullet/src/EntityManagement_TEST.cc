/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/plugin/Loader.hh>

#include <ignition/math/eigen3/Conversions.hh>

#include <ignition/physics/RequestEngine.hh>

#include "EntityManagementFeatures.hh"
#include "JointFeatures.hh"

// ToDo(Lobotuerk): Once more tests are added into this plugin, delete line 35
// and uncomment 31~33 adding another feature list to clear the warn.
// struct TestFeatureList : ignition::physics::FeatureList<
//     ignition::physics::bullet::EntityManagementFeatureList
// > { };

using TestFeatureList = ignition::physics::bullet::EntityManagementFeatureList;

TEST(EntityManagement_TEST, ConstructEmptyWorld)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(bullet_plugin_LIB);

  ignition::plugin::PluginPtr bullet =
      loader.Instantiate("ignition::physics::bullet::Plugin");

  auto engine =
      ignition::physics::RequestEngine3d<TestFeatureList>::From(bullet);
  ASSERT_NE(nullptr, engine);

  auto world = engine->ConstructEmptyWorld("empty world");
  ASSERT_NE(nullptr, world);
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
