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
#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>
#include <gz/plugin/Loader.hh>

#include <gz/physics/FindFeatures.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>

// The features that an engine must have to be loaded by this loader.
using Features = gz::physics::FeatureList<
  gz::physics::GetEngineInfo
>;

class EntityManagementFeaturesTest:
  public testing::Test
{
  public: static void init(int argc, char *argv[])
  {
    if (argc != 2)
      FAIL() << "Please provide the path to an engine plugin.\n"
             << "Usage COMMON_TEST_basic_test <physics engine path>\n";
    libToTest = argv[1];
  }

  static std::string GetLibToTest()
  {
    return libToTest;
  }

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
      FAIL() << "No plugins with required features found in " << GetLibToTest();
    }
  }

  /// \brief Get Physics Engine name based on the plugin name
  /// \param[in] _name Plugin name
  /// \return Name of the Physics Engine
  std::string PhysicsEngineName(std::string _name)
  {
    std::vector<std::string> tokens = gz::common::split(_name, "::");
    if (tokens.size() == 4)
    {
      std::string physicsEngineName = tokens[2];
      std::string physicsEnginePluginName = physicsEngineName;
      if (physicsEngineName == "tpeplugin")
      {
        physicsEnginePluginName = "tpe";
      }
      return physicsEnginePluginName;
    }
    return "";
  }

  public: static std::string libToTest;
  public: gz::plugin::Loader loader;
  public: std::set<std::string> pluginNames;
};

std::string EntityManagementFeaturesTest::libToTest = std::string("");

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
  EntityManagementFeaturesTest::init(argc, argv);
  return RUN_ALL_TESTS();
}
