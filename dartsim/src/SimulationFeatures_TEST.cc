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
#include <set>

#include <ignition/math/Vector3.hh>
#include <ignition/math/eigen3/Conversions.hh>

#include <ignition/physics/FindFeatures.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/physics/RequestEngine.hh>

// Features
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/GetContacts.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Shape.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/PhysicsPluginsList.hh>
#include <test/Utils.hh>

struct TestFeatureList : ignition::physics::FeatureList<
    ignition::physics::LinkFrameSemantics,
    ignition::physics::ForwardStep,
    ignition::physics::GetContactsFromLastStepFeature,
    ignition::physics::GetEntities,
    ignition::physics::GetShapeBoundingBox,
    ignition::physics::CollisionFilterMaskFeature,
    ignition::physics::sdf::ConstructSdfWorld
> { };

using TestWorldPtr = ignition::physics::World3dPtr<TestFeatureList>;
using TestShapePtr = ignition::physics::Shape3dPtr<TestFeatureList>;
using ContactPoint = ignition::physics::World3d<TestFeatureList>::ContactPoint;
using ExtraContactData =
    ignition::physics::World3d<TestFeatureList>::ExtraContactData;

std::unordered_set<TestWorldPtr> LoadWorlds(
    const std::string &_library,
    const std::string &_world)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(resolveLibrary(_library));

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

// Test that the dartsim plugin loaded all the relevant information correctly.
TEST_P(SimulationFeatures_TEST, Falling)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  std::cout << "Testing library " << library << std::endl;
  auto worlds = LoadWorlds(library, TEST_WORLD_DIR "/falling.world");

  for (const auto &world : worlds)
  {
    StepWorld(world, 1000);

    auto link = world->GetModel(0)->GetLink(0);
    auto pos = link->FrameDataRelativeToWorld().pose.translation();
    EXPECT_NEAR(pos.z(), 1.0, 5e-2);
  }
}

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

// Tests collision filtering based on bitmasks
TEST_P(SimulationFeatures_TEST, CollideBitmasks)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  auto worlds = LoadWorlds(library, TEST_WORLD_DIR "/shapes_bitmask.sdf");

  for (const auto &world : worlds)
  {
    auto baseBox = world->GetModel("box_base");
    auto filteredBox = world->GetModel("box_filtered");
    auto collidingBox = world->GetModel("box_colliding");

    StepWorld(world);
    auto contacts = world->GetContactsFromLastStep();
    // Only one box (box_colliding) should collide
    EXPECT_EQ(4u, contacts.size());

    // Now disable collisions for the colliding box as well
    auto collidingShape = collidingBox->GetLink(0)->GetShape(0);
    auto filteredShape = filteredBox->GetLink(0)->GetShape(0);
    collidingShape->SetCollisionFilterMask(0xF0);
    // Also test the getter
    EXPECT_EQ(0xF0, collidingShape->GetCollisionFilterMask());
    // Step and make sure there is no collisions
    StepWorld(world);
    contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(0u, contacts.size());

    // Now remove both filter masks (no collision will be filtered)
    // Equivalent to set to 0xFF
    collidingShape->RemoveCollisionFilterMask();
    filteredShape->RemoveCollisionFilterMask();
    StepWorld(world);
    // Expect both objects to collide
    contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(8u, contacts.size());
  }
}

TEST_P(SimulationFeatures_TEST, RetrieveContacts)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  auto worlds = LoadWorlds(library, TEST_WORLD_DIR "/contact.sdf");


  for (const auto &world : worlds)
  {
    auto sphere = world->GetModel("sphere");
    auto groundPlane = world->GetModel("ground_plane");
    auto groundPlaneCollision = groundPlane->GetLink(0)->GetShape(0);

    // The first step already has contacts, but the contact force due to the
    // impact does not match the steady-state force generated by the
    // body's weight.
    StepWorld(world);

    // After a second step, the contact force reaches steady-state
    StepWorld(world);

    auto contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(4u, contacts.size());

    // Use a set because the order of collisions is not determined.
    std::set<TestShapePtr> possibleCollisions = {
        groundPlaneCollision,
        sphere->GetLink(0)->GetShape(0),
        sphere->GetLink(1)->GetShape(0),
        sphere->GetLink(2)->GetShape(0),
        sphere->GetLink(3)->GetShape(0),
    };
    std::map<TestShapePtr, Eigen::Vector3d> expectations
    {
      {sphere->GetLink(0)->GetShape(0), {0.0, 0.0, 0.0}},
      {sphere->GetLink(1)->GetShape(0), {0.0, 1.0, 0.0}},
      {sphere->GetLink(2)->GetShape(0), {1.0, 0.0, 0.0}},
      {sphere->GetLink(3)->GetShape(0), {1.0, 1.0, 0.0}},
    };

    const double gravity = 9.8;
    std::map<TestShapePtr, double> forceExpectations
    {
      // Contact force expectations are: link mass * gravity.
      {sphere->GetLink(0)->GetShape(0), 0.1 * gravity},
      {sphere->GetLink(1)->GetShape(0), 1.0 * gravity},
      {sphere->GetLink(2)->GetShape(0), 2.0 * gravity},
      {sphere->GetLink(3)->GetShape(0), 3.0 * gravity},
    };

    for (auto &contact : contacts)
    {
      const auto &contactPoint = contact.Get<ContactPoint>();
      ASSERT_TRUE(contactPoint.collision1);
      ASSERT_TRUE(contactPoint.collision2);

      EXPECT_TRUE(possibleCollisions.find(contactPoint.collision1) !=
                  possibleCollisions.end());
      EXPECT_TRUE(possibleCollisions.find(contactPoint.collision2) !=
                  possibleCollisions.end());
      EXPECT_NE(contactPoint.collision1, contactPoint.collision2);

      Eigen::Vector3d expectedContactPos = Eigen::Vector3d::Zero();

      // The test expectations are all on the collision that is not the ground
      // plane.
      auto testCollision = contactPoint.collision1;
      if (testCollision == groundPlaneCollision)
      {
        testCollision = contactPoint.collision2;
      }

      expectedContactPos = expectations.at(testCollision);

      EXPECT_TRUE(ignition::physics::test::Equal(expectedContactPos,
                                                 contactPoint.point, 1e-6));

      // Check if the engine populated the extra contact data struct
      const auto* extraContactData = contact.Query<ExtraContactData>();
      ASSERT_NE(nullptr, extraContactData);

      // The normal of the contact force is a vector pointing up (z positive)
      EXPECT_NEAR(extraContactData->normal[0], 0.0, 1e-3);
      EXPECT_NEAR(extraContactData->normal[1], 0.0, 1e-3);
      EXPECT_NEAR(extraContactData->normal[2], 1.0, 1e-3);

      // The contact force has only a z component and its value is
      // the the weight of the sphere times the gravitational acceleration
      EXPECT_NEAR(extraContactData->force[0], 0.0, 1e-3);
      EXPECT_NEAR(extraContactData->force[1], 0.0, 1e-3);
      EXPECT_NEAR(extraContactData->force[2],
                  forceExpectations.at(testCollision), 1e-3);
    }
  }
}

INSTANTIATE_TEST_SUITE_P(PhysicsPlugins, SimulationFeatures_TEST,
    ::testing::ValuesIn(ignition::physics::test::g_PhysicsPluginLibraries)); // NOLINT

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
