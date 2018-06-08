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

#include <ignition/common/PluginLoader.hh>
#include <ignition/common/SystemPaths.hh>

#include <ignition/physics/RequestFeatures.hh>
#include "../MockFeatures.hh"

using namespace ignition::physics;

using MockFeatureList = FeatureList<mock::MockGetByName>;
using MockEnginePtr = std::unique_ptr<Engine3d<MockFeatureList>>;
using MockWorldPtr = std::unique_ptr<World3d<MockFeatureList>>;
using MockModelPtr = std::unique_ptr<Model3d<MockFeatureList>>;
using MockLinkPtr = std::unique_ptr<Link3d<MockFeatureList>>;
using MockJointPtr = std::unique_ptr<Joint3d<MockFeatureList>>;

#define IGN_MOCK_PLUGIN_PATH \
  DETAIL_IGN_MOCK_PLUGIN_PATH

TEST(FeatureSystem, MockPlugin)
{
  ignition::common::SystemPaths sp;
  sp.AddPluginPaths(IGN_MOCK_PLUGIN_PATH);
  const std::string path = sp.FindSharedLibrary("MockPhysics");

  ignition::common::PluginLoader pl;
  auto plugins = pl.LoadLibrary(path);
  EXPECT_EQ(1u, plugins.size());

  ignition::common::PluginPtr plugin =
      pl.Instantiate("mock::MockPhysicsPlugin");
  EXPECT_TRUE(plugin);


   MockEnginePtr engine =
      ignition::physics::RequestFeatures3d<MockFeatureList>::From(plugin);

  EXPECT_EQ("Only one engine", engine->Name());

  MockWorldPtr world = engine->GetWorld("Some world");
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("Some world", world->Name());

  MockModelPtr model1 = world->GetModel("First model");
  ASSERT_NE(nullptr, model1);
  EXPECT_EQ("First model", model1->Name());

  MockLinkPtr link1_1 = model1->GetLink("First link");
  ASSERT_NE(nullptr, link1_1);
  EXPECT_EQ("First link", link1_1->Name());

  MockJointPtr joint1 = model1->GetJoint("A joint");
  ASSERT_NE(nullptr, joint1);
  EXPECT_EQ("A joint", joint1->Name());

  MockLinkPtr link1_2 = model1->GetLink("Second link");
  ASSERT_NE(nullptr, link1_2);
  EXPECT_EQ("Second link", link1_2->Name());

  MockModelPtr model2 = world->GetModel("Second model");
  ASSERT_NE(nullptr, model2);
  EXPECT_EQ("Second model", model2->Name());

  MockLinkPtr link2 = model2->GetLink("Link of second model");
  ASSERT_NE(nullptr, link2);
  EXPECT_EQ("Link of second model", link2->Name());

  MockJointPtr joint2 = model2->GetJoint("Another joint");
  ASSERT_NE(nullptr, joint2);
  EXPECT_EQ("Another joint", joint2->Name());
}

/////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
