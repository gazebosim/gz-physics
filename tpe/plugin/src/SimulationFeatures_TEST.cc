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

#include <ignition/common/Console.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/eigen3/Conversions.hh>

#include <ignition/physics/FindFeatures.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/physics/RequestEngine.hh>

// Features
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/PhysicsPluginsList.hh>
#include <test/Utils.hh>

#include "EntityManagementFeatures.hh"
#include "ShapeFeatures.hh"
#include "SimulationFeatures.hh"

struct TestFeatureList : ignition::physics::FeatureList<
  ignition::physics::tpeplugin::SimulationFeatureList,
  ignition::physics::tpeplugin::ShapeFeatureList,
  ignition::physics::tpeplugin::EntityManagementFeatureList,
  ignition::physics::LinkFrameSemantics,
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

  EXPECT_EQ(1u, pluginNames.size());

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

void StepWorld(const TestWorldPtr &_world, const std::size_t _num_steps = 1)
{
  ignition::physics::ForwardStep::Input input;
  ignition::physics::ForwardStep::State state;
  ignition::physics::ForwardStep::Output output;

  for (size_t i = 0; i < _num_steps; ++i)
  {
    _world->Step(output, state, input);
  }
}

class SimulationFeatures_TEST
  : public ::testing::Test,
    public ::testing::WithParamInterface<std::string>
{};

// Test that the tpe plugin loaded all the relevant information correctly.
TEST_P(SimulationFeatures_TEST, StepWorld)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  std::cout << "Testing library " << library << std::endl;
  auto worlds = LoadWorlds(library, TEST_WORLD_DIR "/shapes.world");

  for (const auto &world : worlds)
  {
    StepWorld(world, 1000);

    auto link = world->GetModel(0)->GetLink(0);
    auto pos = link->FrameDataRelativeToWorld().pose.translation();
    EXPECT_NEAR(pos.z(), 0.5, 5e-2);
  }
}

TEST_P(SimulationFeatures_TEST, ShapeFeatures)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  auto worlds = LoadWorlds(library, TEST_WORLD_DIR "/shapes.world");

  for (const auto &world : worlds)
  {
    // test ShapeFeatures
    auto sphere = world->GetModel("sphere");
    auto sphereLink = sphere->GetLink(0);
    auto sphereCollision = sphereLink->GetShape(0);
    auto sphereShape = sphereCollision->CastToSphereShape();
    EXPECT_NEAR(1.0, sphereShape->GetRadius(), 1e-6);

    auto sphere2 = sphereLink->AttachSphereShape(
      "sphere2", 1.0, Eigen::Isometry3d::Identity());
    EXPECT_EQ(2u, sphereLink->GetShapeCount());
    EXPECT_EQ(sphere2, sphereLink->GetShape(1));
  
    auto ground = world->GetModel("box");
    auto groundLink = ground->GetLink(0);
    auto groundCollision = groundLink->GetShape(0);
    auto boxShape = groundCollision->CastToBoxShape();
    EXPECT_EQ(ignition::math::Vector3d(100, 100, 1),
              ignition::math::eigen3::convert(boxShape->GetSize()));

    auto box2 = groundLink->AttachBoxShape(
      "box2",
      ignition::math::eigen3::convert(
        ignition::math::Vector3d(1.2, 1.2, 1.2)),
      Eigen::Isometry3d::Identity());
    EXPECT_EQ(2u, groundLink->GetShapeCount());
    EXPECT_EQ(box2, groundLink->GetShape(1));

    auto cylinder = world->GetModel("cylinder");
    auto cylinderLink = cylinder->GetLink(0);
    auto cylinderCollision = cylinderLink->GetShape(0);
    auto cylinderShape = cylinderCollision->CastToCylinderShape();
    EXPECT_NEAR(0.5, cylinderShape->GetRadius(), 1e-6);
    EXPECT_NEAR(1.1, cylinderShape->GetHeight(), 1e-6);

    auto cylinder2 = cylinderLink->AttachCylinderShape(
      "cylinder2", 3.0, 4.0, Eigen::Isometry3d::Identity());
    EXPECT_EQ(2u, cylinderLink->GetShapeCount());
    EXPECT_EQ(cylinder2, cylinderLink->GetShape(1));

    // Test the bounding boxes in the local frames
    auto sphereAABB =
      sphereCollision->GetAxisAlignedBoundingBox(*sphereCollision);

    auto groundAABB =
      groundCollision->GetAxisAlignedBoundingBox(*groundCollision);
    
    auto cylinderAABB =
      cylinderCollision->GetAxisAlignedBoundingBox(*cylinderCollision);

    EXPECT_EQ(ignition::math::Vector3d(-1, -1, -1),
              ignition::math::eigen3::convert(sphereAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(1, 1, 1),
              ignition::math::eigen3::convert(sphereAABB).Max());
    EXPECT_EQ(ignition::math::Vector3d(-50, -50, -0.5),
              ignition::math::eigen3::convert(groundAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(50, 50, 0.5),
              ignition::math::eigen3::convert(groundAABB).Max());
    EXPECT_EQ(ignition::math::Vector3d(-0.5, -0.5, -0.55),
              ignition::math::eigen3::convert(cylinderAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(0.5, 0.5, 0.55),
              ignition::math::eigen3::convert(cylinderAABB).Max());
  }
}

INSTANTIATE_TEST_CASE_P(PhysicsPlugins, SimulationFeatures_TEST,
  ::testing::ValuesIn(ignition::physics::test::g_PhysicsPluginLibraries),); // NOLINT

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
