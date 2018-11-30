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
#include <ignition/physics/Link.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/PhysicsPluginsList.hh>

using TestFeatureList = ignition::physics::FeatureList<
  ignition::physics::SetLinkState,
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
    const sdf::Errors errors = root.Load(_world);
    EXPECT_TRUE(errors.empty());
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    auto world = engine->ConstructWorld(*sdfWorld);

    worlds.insert(world);
  }

  return worlds;
}

class LinkFeatures_TEST
  : public ::testing::Test,
    public ::testing::WithParamInterface<std::string>
{};

INSTANTIATE_TEST_CASE_P(PhysicsPlugins, LinkFeatures_TEST,
    ::testing::ValuesIn(ignition::physics::test::g_PhysicsPluginLibraries),); // NOLINT

// Test that linear and angular velocities of a link can be set.
TEST_P(LinkFeatures_TEST, LinkVelocity)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  std::cout << "Testing library " << library << std::endl;
  auto worlds = LoadWorlds(library, TEST_WORLD_DIR "/falling.world");

  for (const auto &world : worlds)
  {
    auto link = world->GetModel(0)->GetLink(0);
    Eigen::Vector3d initPos = 
        link->FrameDataRelativeToWorld().pose.translation();
    link->SetLinearVelocity({1, -1, 10});

    ignition::physics::ForwardStep::Input input;
    ignition::physics::ForwardStep::State state;
    ignition::physics::ForwardStep::Output output;

    // assumes step size is 1ms
    for (size_t i = 0; i < 1000; ++i)
    {
      world->Step(output, state, input);
    }

    Eigen::Vector3d pos = link->FrameDataRelativeToWorld().pose.translation();
    EXPECT_NEAR(1.0, pos.x() - initPos.x(),  5e-2);
    EXPECT_NEAR(-1.0, pos.y() - initPos.y(), 5e-2);
    const double gravity = -9.8;
    const double zExpected = 10 + 0.5 * gravity;
    EXPECT_NEAR(zExpected, pos.z() - initPos.z(), 5e-2);
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
