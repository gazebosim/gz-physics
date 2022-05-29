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

#include <gz/plugin/Loader.hh>

#include <gz/physics/RequestEngine.hh>
#include "../MockFeatures.hh"

using namespace gz::physics;

/////////////////////////////////////////////////
gz::plugin::PluginPtr LoadMockPlugin(const std::string &_pluginName)
{
  gz::plugin::Loader pl;
  auto plugins = pl.LoadLib(MockEntities_LIB);
  EXPECT_EQ(2u, plugins.size());

  gz::plugin::PluginPtr plugin =
      pl.Instantiate(_pluginName);
  EXPECT_TRUE(plugin);

  return plugin;
}

/////////////////////////////////////////////////
TEST(FeatureSystem, MockPluginInvalidFeatures)
{
  // mismatch between 2d and 3d
  mock::MockEngine3dPtr engine =
      gz::physics::RequestEngine3d<mock::MockFeatureList>::From(
         LoadMockPlugin("mock::EntitiesPlugin2d"));
  EXPECT_EQ(nullptr, engine);
}

/////////////////////////////////////////////////
TEST(FeatureSystem, MockPlugin)
{
  mock::MockEngine3dPtr engine =
      gz::physics::RequestEngine3d<mock::MockFeatureList>::From(
         LoadMockPlugin("mock::EntitiesPlugin3d"));

  EXPECT_EQ("Only one engine", engine->Name());

  mock::MockWorld3dPtr world = engine->GetWorld("Some world");
  ASSERT_NE(nullptr, world);
  EXPECT_EQ("Some world", world->Name());

  mock::MockModel3dPtr model1 = world->GetModel("First model");
  ASSERT_NE(nullptr, model1);
  EXPECT_EQ("First model", model1->Name());
  // The Entity ID of the model objects that get returned should be consistent.
  EXPECT_EQ(model1->EntityID(), world->GetModel("First model")->EntityID());
  // Our mock plugin does not do reference counting, so we expect the reference
  // to be a nullptr.
  EXPECT_EQ(nullptr, model1->EntityReference());

  EXPECT_TRUE(model1->SetName("Changed model name"));
  EXPECT_EQ("Changed model name", model1->Name());

  // There should no longer be a model named "First model"
  EXPECT_EQ(nullptr, world->GetModel("First model"));

  mock::MockLink3dPtr link1_1 = model1->GetLink("First link");
  ASSERT_NE(nullptr, link1_1);
  EXPECT_EQ("First link", link1_1->Name());

  mock::MockJoint3dPtr joint1 = model1->GetJoint("A joint");
  ASSERT_NE(nullptr, joint1);
  EXPECT_EQ("A joint", joint1->Name());

  mock::MockLink3dPtr link1_2 = model1->GetLink("Second link");
  ASSERT_NE(nullptr, link1_2);
  EXPECT_EQ("Second link", link1_2->Name());

  EXPECT_TRUE(link1_2->SetName("Changed link name"));
  EXPECT_EQ("Changed link name", link1_2->Name());

  EXPECT_FALSE(link1_1->SetName("Changed link name"));
  EXPECT_EQ("First link", link1_1->Name());

  mock::MockModel3dPtr model2 = world->GetModel("Second model");
  ASSERT_NE(nullptr, model2);
  EXPECT_EQ("Second model", model2->Name());

  EXPECT_FALSE(model2->SetName("Changed model name"));
  EXPECT_EQ("Second model", model2->Name());
  EXPECT_TRUE(model2->SetName("Second model"));
  EXPECT_NE(nullptr, world->GetModel("Second model"));

  mock::MockLink3dPtr link2 = model2->GetLink("Link of second model");
  ASSERT_NE(nullptr, link2);
  EXPECT_EQ("Link of second model", link2->Name());

  mock::MockJoint3dPtr joint2 = model2->GetJoint("Another joint");
  ASSERT_NE(nullptr, joint2);
  EXPECT_EQ("Another joint", joint2->Name());
}

/////////////////////////////////////////////////
TEST(FeatureSystem, MockCenterOfMass3d)
{
  mock::MockEngine3dPtr engine =
      gz::physics::RequestEngine3d<mock::MockFeatureList>::From(
        LoadMockPlugin("mock::EntitiesPlugin3d"));

  mock::MockWorld3dPtr world = engine->GetWorld("Some world");
  ASSERT_NE(nullptr, world);

  mock::MockModel3dPtr model1 = world->GetModel("First model");
  ASSERT_NE(nullptr, model1);

  mock::MockLink3dPtr link1_1 = model1->GetLink("First link");
  ASSERT_NE(nullptr, link1_1);
  Eigen::Vector3d com1_1 = link1_1->CenterOfMass();
  EXPECT_DOUBLE_EQ(1.0, com1_1[0]);
  EXPECT_DOUBLE_EQ(0.0, com1_1[1]);
  EXPECT_DOUBLE_EQ(3.0, com1_1[2]);

  mock::MockLink3dPtr link1_2 = model1->GetLink("Second link");
  ASSERT_NE(nullptr, link1_2);
  const Eigen::Vector3d com1_2 = link1_2->CenterOfMass();
  EXPECT_DOUBLE_EQ(0.0, com1_2[0]);
  EXPECT_DOUBLE_EQ(2.0, com1_2[1]);
  EXPECT_DOUBLE_EQ(4.0, com1_2[2]);

  const Eigen::Vector3d com1 = model1->CenterOfMass();
  const Eigen::Vector3d expectedCom1 = (com1_1 + com1_2)/2.0;
  EXPECT_DOUBLE_EQ(expectedCom1[0], com1[0]);
  EXPECT_DOUBLE_EQ(expectedCom1[1], com1[1]);
  EXPECT_DOUBLE_EQ(expectedCom1[2], com1[2]);

  mock::MockModel3dPtr model2 = world->GetModel("Second model");
  ASSERT_NE(nullptr, model2);

  mock::MockLink3dPtr link2_1 = model2->GetLink("Link of second model");
  ASSERT_NE(nullptr, link2_1);
  const Eigen::Vector3d com2_1 = link2_1->CenterOfMass();
  EXPECT_DOUBLE_EQ(5.0, com2_1[0]);
  EXPECT_DOUBLE_EQ(0.0, com2_1[1]);
  EXPECT_DOUBLE_EQ(8.0, com2_1[2]);

  const Eigen::Vector3d com2 = model2->CenterOfMass();
  const Eigen::Vector3d expectedCom2 = com2_1;
  EXPECT_DOUBLE_EQ(expectedCom2[0], com2[0]);
  EXPECT_DOUBLE_EQ(expectedCom2[1], com2[1]);
  EXPECT_DOUBLE_EQ(expectedCom2[2], com2[2]);
}

/////////////////////////////////////////////////
TEST(FeatureSystem, MockCenterOfMass2d)
{
  mock::MockEngine2dPtr engine =
      gz::physics::RequestEngine2d<mock::MockFeatureList>::From(
        LoadMockPlugin("mock::EntitiesPlugin2d"));

  mock::MockWorld2dPtr world = engine->GetWorld("Some world");
  ASSERT_NE(nullptr, world);

  mock::MockModel2dPtr model1 = world->GetModel("First model");
  ASSERT_NE(nullptr, model1);

  mock::MockLink2dPtr link1_1 = model1->GetLink("First link");
  ASSERT_NE(nullptr, link1_1);
  Eigen::Vector2d com1_1 = link1_1->CenterOfMass();
  EXPECT_DOUBLE_EQ(1.0, com1_1[0]);
  EXPECT_DOUBLE_EQ(0.0, com1_1[1]);

  mock::MockLink2dPtr link1_2 = model1->GetLink("Second link");
  ASSERT_NE(nullptr, link1_2);
  const Eigen::Vector2d com1_2 = link1_2->CenterOfMass();
  EXPECT_DOUBLE_EQ(0.0, com1_2[0]);
  EXPECT_DOUBLE_EQ(2.0, com1_2[1]);

  const Eigen::Vector2d com1 = model1->CenterOfMass();
  const Eigen::Vector2d expectedCom1 = (com1_1 + com1_2)/2.0;
  EXPECT_DOUBLE_EQ(expectedCom1[0], com1[0]);
  EXPECT_DOUBLE_EQ(expectedCom1[1], com1[1]);

  mock::MockModel2dPtr model2 = world->GetModel("Second model");
  ASSERT_NE(nullptr, model2);

  mock::MockLink2dPtr link2_1 = model2->GetLink("Link of second model");
  ASSERT_NE(nullptr, link2_1);
  const Eigen::Vector2d com2_1 = link2_1->CenterOfMass();
  EXPECT_DOUBLE_EQ(5.0, com2_1[0]);
  EXPECT_DOUBLE_EQ(0.0, com2_1[1]);

  const Eigen::Vector2d com2 = model2->CenterOfMass();
  const Eigen::Vector2d expectedCom2 = com2_1;
  EXPECT_DOUBLE_EQ(expectedCom2[0], com2[0]);
  EXPECT_DOUBLE_EQ(expectedCom2[1], com2[1]);
}

/////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
