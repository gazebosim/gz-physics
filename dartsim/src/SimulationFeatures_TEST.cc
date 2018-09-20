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

#include <iostream>

#include <gtest/gtest.h>

#include <ignition/plugin/Loader.hh>
#include <ignition/physics/RequestEngine.hh>

// Features
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

using TestFeatureList = ignition::physics::FeatureList<
  ignition::physics::LinkFrameSemantics,
  ignition::physics::ForwardStep,
  ignition::physics::GetEntities,
  ignition::physics::sdf::ConstructSdfWorld
>;

auto LoadWorld(const std::string &_world)
{
  ignition::plugin::Loader loader;
  loader.LoadLibrary(dartsim_plugin_LIB);

  ignition::plugin::PluginPtr dartsim =
      loader.Instantiate("ignition::physics::dartsim::Plugin");

  auto engine =
      ignition::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
  EXPECT_NE(nullptr, engine);

  sdf::Root root;
  const sdf::Errors &errors = root.Load(_world);
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  auto world = engine->ConstructWorld(*sdfWorld);

  return world;
}

// Test that the dartsim plugin loaded all the relevant information correctly.
TEST(SimulationFeatures_TEST, Falling)
{
  auto world = LoadWorld(TEST_WORLD_DIR "/falling.world");

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
  EXPECT_NEAR(pos.z(), 1.0, 1e-2);
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
