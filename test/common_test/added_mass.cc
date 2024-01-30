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

#include "test/TestLibLoader.hh"
#include "test/Utils.hh"
#include "Worlds.hh"

#include <gz/physics/AddedMass.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>

#include <gz/physics/World.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>

using AssertVectorApprox = gz::physics::test::AssertVectorApprox;

// The features that an engine must have to be loaded by this loader.
using AddedMassFeatures = gz::physics::FeatureList<
  gz::physics::AddedMass,
  gz::physics::GetEngineInfo,
  gz::physics::Gravity,
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::LinkFrameSemantics,
  gz::physics::GetModelFromWorld,
  gz::physics::GetLinkFromModel,
  gz::physics::ForwardStep
>;

class AddedMassFeaturesTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    auto plugins = loader.LoadLib(AddedMassFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<AddedMassFeatures>::From(loader);
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
TEST_F(AddedMassFeaturesTest, Gravity)
{
  for (const std::string &name : this->pluginNames)
  {
    std::cout << "Testing plugin: " << name << std::endl;
    gz::plugin::PluginPtr plugin = this->loader.Instantiate(name);

    auto engine = gz::physics::RequestEngine3d<AddedMassFeatures>::From(plugin);
    ASSERT_NE(nullptr, engine);
    EXPECT_TRUE(engine->GetName().find(this->PhysicsEngineName(name)) !=
                std::string::npos);

    sdf::Root root;
    const sdf::Errors errors = root.Load(
      common_test::worlds::kFallingAddedMassWorld);
    EXPECT_TRUE(errors.empty()) << errors;
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    EXPECT_NE(nullptr, sdfWorld);

    auto world = engine->ConstructWorld(*root.WorldByIndex(0));
    EXPECT_NE(nullptr, world);

    auto graphErrors = sdfWorld->ValidateGraphs();
    EXPECT_EQ(0u, graphErrors.size()) << graphErrors;

    AssertVectorApprox vectorPredicate6(1e-6);

    // Set gravity to a nice round number
    world->SetGravity({0, 0, -10});

    // Link poses
    const Eigen::Vector3d initialLinkPosition(0, 0, 2);
    const Eigen::Vector3d finalLinkPosition(0, 0, -3.005);
    const Eigen::Vector3d finalLinkPositionAddedMass(0, 0, -0.5025);

    // This tests that the physics plugin correctly considers added mass.
    for (auto modelName: {"sphere", "sphere_zero_added_mass", "sphere_added_mass", "heavy_sphere"})
    {
      auto model = world->GetModel(modelName);
      ASSERT_NE(nullptr, model);

      auto link = model->GetLink(0);
      ASSERT_NE(nullptr, link);

      Eigen::Vector3d pos = link->FrameDataRelativeToWorld().pose.translation();
      EXPECT_PRED_FORMAT2(vectorPredicate6,
                          initialLinkPosition,
                          pos);
    }

    gz::physics::ForwardStep::Input input;
    gz::physics::ForwardStep::State state;
    gz::physics::ForwardStep::Output output;

    const size_t numSteps = 1000;
    for (size_t i = 0; i < numSteps; ++i)
    {
      world->Step(output, state, input);
    }

    // Confirm that the models with zero added mass behave consistently
    for (auto modelName: {"sphere", "sphere_zero_added_mass", "heavy_sphere"})
    {
      auto model = world->GetModel(modelName);
      ASSERT_NE(nullptr, model);

      auto link = model->GetLink(0);
      ASSERT_NE(nullptr, link);

      Eigen::Vector3d pos = link->FrameDataRelativeToWorld().pose.translation();
      EXPECT_PRED_FORMAT2(vectorPredicate6,
                          finalLinkPosition,
                          pos);
    }

    for (auto modelName: {"sphere_added_mass"})
    {
      auto model = world->GetModel(modelName);
      ASSERT_NE(nullptr, model);

      auto link = model->GetLink(0);
      ASSERT_NE(nullptr, link);

      Eigen::Vector3d pos = link->FrameDataRelativeToWorld().pose.translation();
      EXPECT_PRED_FORMAT2(vectorPredicate6,
                          finalLinkPositionAddedMass,
                          pos);
    }
  }

}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  if (!AddedMassFeaturesTest::init(argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
