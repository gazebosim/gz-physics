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

#include <gz/physics/FindFeatures.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>

// The features that an engine must have to be loaded by this loader.
using Features = gz::physics::FeatureList<
  gz::physics::GetEngineInfo
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
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!EntityManagementFeaturesTest::init(argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
