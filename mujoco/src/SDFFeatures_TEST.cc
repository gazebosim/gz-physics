/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <gz/physics/GetEntities.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <test/Utils.hh>

#include "test/common_test/Worlds.hh"

using namespace gz;

struct TestFeatureList : physics::FeatureList<
    physics::GetEngineInfo,
    physics::GetWorldFromEngine,
    physics::GetModelFromWorld,
    physics::sdf::ConstructSdfModel,
    physics::sdf::ConstructSdfWorld,
    physics::RemoveEntities
> { };

using World = physics::World3d<TestFeatureList>;
using WorldPtr = physics::World3dPtr<TestFeatureList>;
using ModelPtr = physics::Model3dPtr<TestFeatureList>;
using LinkPtr = physics::Link3dPtr<TestFeatureList>;

/////////////////////////////////////////////////
auto LoadEngine()
{
  plugin::Loader loader;
  loader.LoadLib(mujoco_plugin_LIB);

  plugin::PluginPtr mujoco =
      loader.Instantiate("gz::physics::mujoco::Plugin");

  auto engine =
      physics::RequestEngine3d<TestFeatureList>::From(mujoco);
  return engine;
}

enum class LoaderType
{
  Whole,
  Piecemeal
};

/////////////////////////////////////////////////
WorldPtr LoadWorldWhole(const std::string &_world)
{
  auto engine = LoadEngine();
  EXPECT_NE(nullptr, engine);

  sdf::Root root;
  const sdf::Errors &errors = root.Load(_world);
  EXPECT_EQ(0u, errors.size());
  for (const auto & error : errors) {
    std::cout << error << std::endl;
  }

  EXPECT_EQ(1u, root.WorldCount());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  EXPECT_NE(nullptr, sdfWorld);

  auto world = engine->ConstructWorld(*sdfWorld);
  EXPECT_NE(nullptr, world);

  return world;
}

/////////////////////////////////////////////////
class SDFFeatures_TEST : public ::testing::TestWithParam<LoaderType>
{
  public: WorldPtr LoadWorld(const std::string &_world)
  {
    switch(this->GetParam())
    {
      case LoaderType::Whole:
        return LoadWorldWhole(_world);
      // case LoaderType::Piecemeal:
      //   return LoadWorldPiecemeal(_world);
      default:
        std::cout << "Unknown LoaderType "
                  << std::underlying_type_t<LoaderType>(this->GetParam())
                  << " Using LoadWorldWhole" << std::endl;
        return LoadWorldWhole(_world);
    }
  }
};
/////////////////////////////////////////////////
// Test that the mujoco plugin loaded all the relevant information correctly.
TEST_P(SDFFeatures_TEST, CheckMujocoData)
{
  WorldPtr world = this->LoadWorld(common_test::worlds::kShapesWorld);
  ASSERT_NE(nullptr, world);

  // Check the number of models
  EXPECT_EQ(6u, world->GetModelCount());
}

/////////////////////////////////////////////////
// Test that models can be removed from the world.
TEST_P(SDFFeatures_TEST, ModelRemoval)
{
  WorldPtr world = this->LoadWorld(common_test::worlds::kShapesWorld);
  ASSERT_NE(nullptr, world);

  std::size_t modelCount = world->GetModelCount();
  EXPECT_GT(modelCount, 0u);

  // Get a specific model
  const std::string modelName = "box";
  ModelPtr model = world->GetModel(modelName);
  ASSERT_NE(nullptr, model);

  EXPECT_TRUE(world->RemoveModel(model));

  // The model should be removed from the world's model list immediately
  EXPECT_EQ(modelCount - 1, world->GetModelCount());

  // Try to get it again, should return nullptr
  EXPECT_EQ(nullptr, world->GetModel(modelName));

  // Try to remove it again, should return false
  EXPECT_FALSE(world->RemoveModel(modelName));
}

INSTANTIATE_TEST_SUITE_P(LoadWorld, SDFFeatures_TEST,
                        ::testing::Values(LoaderType::Whole));
