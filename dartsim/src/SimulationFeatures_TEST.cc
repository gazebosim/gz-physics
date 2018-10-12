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

#include <iostream>

#include <ignition/physics/FindFeatures.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/physics/RequestEngine.hh>

// Features
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/PhysicsPluginsList.hh>

using TestFeatureList = ignition::physics::FeatureList<
  ignition::physics::LinkFrameSemantics,
  ignition::physics::ForwardStep,
  ignition::physics::GetEntities,
  ignition::physics::sdf::ConstructSdfWorld
>;

using TestWorldPtr = ignition::physics::World3dPtr<TestFeatureList>;

std::unordered_set<TestWorldPtr> LoadWorlds(
    const std::string &_library,
    const std::string &_world)
{
  ignition::plugin::Loader loader;
  loader.LoadLibrary(_library);

  const std::set<std::string> pluginNames =
      ignition::physics::FindFeatures3d<TestFeatureList>::From(loader);

  std::unordered_set<TestWorldPtr> worlds;
  for (const std::string &name : pluginNames)
  {
    ignition::plugin::PluginPtr plugin = loader.Instantiate(name);

    std::cout << " -- Plugin name: " << name << std::endl;

    auto engine =
        ignition::physics::RequestEngine3d<TestFeatureList>::From(plugin);
    EXPECT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors &errors = root.Load(_world);
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    auto world = engine->ConstructWorld(*sdfWorld);

    worlds.insert(world);
  }

  return worlds;
}

class SimulationFeatures_TEST
  : public ::testing::Test,
    public ::testing::WithParamInterface<std::string>
{};

INSTANTIATE_TEST_CASE_P(PhysicsPlugins, SimulationFeatures_TEST,
    ::testing::ValuesIn(ignition::physics::test::g_PhysicsPluginLibraries),);

// Test that the dartsim plugin loaded all the relevant information correctly.
TEST_P(SimulationFeatures_TEST, Falling)
{
  const std::string library = GetParam();
  if(library.empty())
    return;

  std::cout << "Testing library " << library << std::endl;
  auto worlds = LoadWorlds(library, TEST_WORLD_DIR "/falling.world");

  for (const auto &world : worlds)
  {
    ignition::physics::ForwardStep::Input input;
    ignition::physics::ForwardStep::State state;
    ignition::physics::ForwardStep::Output output;

    for (size_t i = 0; i < 1000; ++i)
    {
      world->Step(output, state, input);
      // for (size_t j = 0; j < world->GetModelCount(); ++j)
      // {
      //   auto model = world->GetModel(j);
      //   auto link = model->GetLink(0);
      //   auto pos =
      //       link->FrameDataRelativeToWorld().pose.translation().transpose();
      //   std::cout << model->GetName() << ": " << pos << std::endl;
      // }
    }

    auto link = world->GetModel(0)->GetLink(0);
    auto pos = link->FrameDataRelativeToWorld().pose.translation();
    EXPECT_NEAR(pos.z(), 1.0, 5e-2);
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
