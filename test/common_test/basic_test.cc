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

    if (!gz::common::env("PHYSICS_ENGINE_NAME", physicsEngineName))
    {
      FAIL();
    }
    std::string libToTest;
    if (!gz::common::env("LIB_TO_TEST", libToTest))
    {
      FAIL();
    }

    gz::plugin::Loader loader;
    loader.LoadLib(libToTest);

    std::string physicsEnginePluginName = physicsEngineName;
    if (physicsEngineName == "tpe")
    {
      physicsEnginePluginName = "tpeplugin";
    }
    physicsPlugin =
        loader.Instantiate("gz::physics::" +
                           physicsEnginePluginName +
                           "::Plugin");
  }

  public: gz::plugin::PluginPtr physicsPlugin;
  public: std::string physicsEngineName;
};

// The features that an engine must have to be loaded by this loader.
using Features = gz::physics::FeatureList<
  gz::physics::GetEngineInfo
>;

/////////////////////////////////////////////////
TEST_F(EntityManagementFeaturesTest, ConstructEmptyWorld)
{
  auto engine =
    gz::physics::RequestEngine3d<Features>::From(physicsPlugin);

  ASSERT_NE(nullptr, engine);
  EXPECT_TRUE(engine->GetName().find(physicsEngineName) != std::string::npos);
}
