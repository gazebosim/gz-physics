/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <gz/common/Console.hh>
#include <gz/plugin/Loader.hh>

#include "../helpers/TestLibLoader.hh"

#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>

// The features that an engine must have to be loaded by this loader.
using Features = gz::physics::FeatureList<
  gz::physics::GetEngineInfo,
  gz::physics::GetWorldFromEngine,
  gz::physics::ConstructEmptyWorldFeature,
  gz::physics::ConstructEmptyModelFeature,
  gz::physics::ConstructEmptyLinkFeature,
  gz::physics::GetModelFromWorld,
  gz::physics::GetLinkFromModel,
  gz::physics::GetShapeFromLink
>;

class EntityManagementFeaturesTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    auto plugins = loader.LoadLib(EntityManagementFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<Features>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in " << GetLibToTest();
      GTEST_SKIP();
    }
  }

  public: std::set<std::string> pluginNames;
  public: gz::plugin::Loader loader;
};

/////////////////////////////////////////////////
TEST_F(EntityManagementFeaturesTest, ConstructEmptyWorld)
{
  for (const std::string &name : pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<Features>::From(plugin);
    ASSERT_NE(nullptr, engine);
    EXPECT_TRUE(engine->GetName().find(PhysicsEngineName(name)) !=
                std::string::npos);

    auto world = engine->ConstructEmptyWorld("empty world");
    ASSERT_NE(nullptr, world);

    EXPECT_EQ(engine, world->GetEngine());
    EXPECT_EQ(world, engine->GetWorld(0));
    EXPECT_EQ(world, engine->GetWorld("empty world"));

    auto model = world->ConstructEmptyModel("empty model");
    EXPECT_EQ(1u, world->GetModelCount());
    ASSERT_NE(nullptr, model);
    EXPECT_EQ("empty model", model->GetName());
    ASSERT_NE(model, world->ConstructEmptyModel("dummy"));
    EXPECT_EQ(2u, world->GetModelCount());
    EXPECT_EQ(0u, model->GetIndex());
    EXPECT_EQ(0u, model->GetLinkCount());

    EXPECT_EQ(world, model->GetWorld());
    EXPECT_EQ(model, world->GetModel(0));
    EXPECT_EQ(model, world->GetModel("empty model"));

    auto link = model->ConstructEmptyLink("empty link");
    EXPECT_EQ(1u, model->GetLinkCount());
    ASSERT_NE(nullptr, link);
    EXPECT_EQ("empty link", link->GetName());
    ASSERT_NE(link, model->ConstructEmptyLink("dummy"));
    EXPECT_EQ(2u, model->GetLinkCount());
    EXPECT_EQ(0u, link->GetIndex());
    EXPECT_EQ(0u, link->GetShapeCount());

    EXPECT_EQ(model, link->GetModel());
    EXPECT_EQ(link, model->GetLink(0));
    EXPECT_EQ(link, model->GetLink("empty link"));
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!EntityManagementFeaturesTest::init(argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
