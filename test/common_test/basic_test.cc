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

#include "test_common_config.h"  // NOLINT(build/include)

#include <gz/common/Filesystem.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Loader.hh>

#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>

class EntityManagementFeaturesTest:
  public testing::Test,
  public testing::WithParamInterface<const char *>
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    gz::plugin::Loader loader;
    std::string pluginPath = gz::common::joinPaths(GZ_PHYSICS_TEST_PLUGIN_PATH,
      std::string("libignition-physics6-") + std::string(GetParam()) +
      std::string("-plugin.so"));
    loader.LoadLib(pluginPath);

    std::string physicsPluginName = GetParam();
    if (std::string(GetParam()) == "tpe")
    {
      physicsPluginName = GetParam() + std::string("plugin");
    }

    physicsPlugin =
      loader.Instantiate(std::string("gz::physics::") +
                         physicsPluginName +
                         std::string("::Plugin"));
  }

  public: gz::plugin::PluginPtr physicsPlugin;
};

// The features that an engine must have to be loaded by this loader.
using Features = gz::physics::FeatureList<
  gz::physics::GetEngineInfo
>;

/////////////////////////////////////////////////
TEST_P(EntityManagementFeaturesTest, ConstructEmptyWorld)
{
  auto engine =
    gz::physics::RequestEngine3d<Features>::From(physicsPlugin);

  ASSERT_NE(nullptr, engine);
  EXPECT_TRUE(engine->GetName().find(GetParam()) != std::string::npos);
}

INSTANTIATE_TEST_CASE_P(EntityManagementFeatures, EntityManagementFeaturesTest,
    PHYSICS_ENGINE_VALUES,
    gz::physics::PrintToStringParam());
