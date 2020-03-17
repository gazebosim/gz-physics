/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <ignition/math/Vector3.hh>
#include <ignition/math/eigen3/Conversions.hh>

#include <ignition/physics/FindFeatures.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/physics/RequestEngine.hh>

// Features
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Shape.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/PhysicsPluginsList.hh>
#include <test/Utils.hh>

struct TestFeatureList : ignition::physics::FeatureList<
  ignition::physics::ForwardStep,
  ignition::physics::GetEngineInfo,
  ignition::physics::GetWorldFromEngine,
  ignition::physics::GetModelFromWorld,
  ignition::physics::GetLinkFromModel,
  ignition::physics::GetShapeFromLink,
  ignition::physics::GetShapeBoundingBox,
  ignition::physics::sdf::ConstructSdfWorld
> { };

using TestWorldPtr = ignition::physics::World3dPtr<TestFeatureList>;
using TestShapePtr = ignition::physics::Shape3dPtr<TestFeatureList>;

std::unordered_set<TestWorldPtr> LoadWorlds(
    const std::string &_library,
    const std::string &_world)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(_library);

  const std::set<std::string> pluginNames =
    ignition::physics::FindFeatures3d<TestFeatureList>::From(loader);

  EXPECT_LT(0u, pluginNames.size());

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
    EXPECT_EQ(0u, errors.size());
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

TEST_P(SimulationFeatures_TEST, ShapeBoundingBox)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  auto worlds = LoadWorlds(library, TEST_WORLD_DIR "/falling.world");

  for (const auto &world : worlds)
  {
    auto sphere = world->GetModel("sphere");
    auto sphereCollision = sphere->GetLink(0)->GetShape(0);
    auto ground = world->GetModel("box");
    auto groundCollision = ground->GetLink(0)->GetShape(0);

    // Test the bounding boxes in the local frames
    auto sphereAABB =
      sphereCollision->GetAxisAlignedBoundingBox(*sphereCollision);

    auto groundAABB =
      groundCollision->GetAxisAlignedBoundingBox(*groundCollision);

    EXPECT_EQ(ignition::math::Vector3d(-1, -1, -1),
              ignition::math::eigen3::convert(sphereAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(1, 1, 1),
              ignition::math::eigen3::convert(sphereAABB).Max());
    EXPECT_EQ(ignition::math::Vector3d(-50, -50, -0.5),
              ignition::math::eigen3::convert(groundAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(50, 50, 0.5),
              ignition::math::eigen3::convert(groundAABB).Max());

    // Test the bounding boxes in the world frames
    sphereAABB = sphereCollision->GetAxisAlignedBoundingBox();
    groundAABB = groundCollision->GetAxisAlignedBoundingBox();

    // The sphere shape has a radius of 1.0, so its bounding box will have
    // dimensions of 1.0 x 1.0 x 1.0. When that bounding box is transformed by
    // a 45-degree rotation, the dimensions that are orthogonal to the axis of
    // rotation will dilate from 1.0 to sqrt(2).
    const double d = std::sqrt(2);
    EXPECT_EQ(ignition::math::Vector3d(-d, -1, 2.0 - d),
              ignition::math::eigen3::convert(sphereAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(d, 1, 2 + d),
              ignition::math::eigen3::convert(sphereAABB).Max());
    EXPECT_EQ(ignition::math::Vector3d(-50*d, -50*d, -1),
              ignition::math::eigen3::convert(groundAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(50*d, 50*d, 0),
              ignition::math::eigen3::convert(groundAABB).Max());
  }
}

INSTANTIATE_TEST_CASE_P(PhysicsPlugins, SimulationFeatures_TEST,
  ::testing::ValuesIn(ignition::physics::test::g_PhysicsPluginLibraries),); // NOLINT

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
