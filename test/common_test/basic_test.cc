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

#include <gz/common/Filesystem.hh>
#include <gz/common/Console.hh>
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
  public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    std::string libToTest;
    if (!gz::common::env("LIB_TO_TEST", libToTest))
    {
      FAIL();
    }

    auto plugins = loader.LoadLib(libToTest);

    pluginNames = gz::physics::FindFeatures3d<Features>::From(loader);
    if (pluginNames.empty())
    {
      FAIL() << "No plugins with required features found in " << libToTest;
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

  public: gz::plugin::Loader loader;
  public: std::set<std::string> pluginNames;
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
