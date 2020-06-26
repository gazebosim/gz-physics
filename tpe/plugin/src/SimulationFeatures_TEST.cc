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

#include <ignition/plugin/Loader.hh>

// Features
#include <ignition/physics/FindFeatures.hh>
#include <ignition/physics/GetBoundingBox.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/RequestEngine.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/PhysicsPluginsList.hh>
#include <test/Utils.hh>

#include "EntityManagementFeatures.hh"
#include "FreeGroupFeatures.hh"
#include "ShapeFeatures.hh"
#include "SimulationFeatures.hh"

struct TestFeatureList : ignition::physics::FeatureList<
  ignition::physics::tpeplugin::SimulationFeatureList,
  ignition::physics::tpeplugin::ShapeFeatureList,
  ignition::physics::tpeplugin::EntityManagementFeatureList,
  ignition::physics::tpeplugin::FreeGroupFeatureList,
  ignition::physics::GetContactsFromLastStepFeature,
  ignition::physics::LinkFrameSemantics,
  ignition::physics::GetModelBoundingBox,
  ignition::physics::sdf::ConstructSdfWorld
> { };

using TestWorldPtr = ignition::physics::World3dPtr<TestFeatureList>;
using TestShapePtr = ignition::physics::Shape3dPtr<TestFeatureList>;
using ContactPoint = ignition::physics::World3d<TestFeatureList>::ContactPoint;

std::unordered_set<TestWorldPtr> LoadWorlds(
    const std::string &_library,
    const std::string &_world)
{
  ignition::plugin::Loader loader;
  loader.LoadLib(resolveLibrary(_library));

  const std::set<std::string> pluginNames =
    ignition::physics::FindFeatures3d<TestFeatureList>::From(loader);

  EXPECT_EQ(1u, pluginNames.size());

  std::unordered_set<TestWorldPtr> worlds;
  for (const std::string &name : pluginNames)
  {
    ignition::plugin::PluginPtr plugin = loader.Instantiate(name);

    igndbg << " -- Plugin name: " << name << std::endl;

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

  igndbg << "Testing library " << library << std::endl;
  auto worlds = LoadWorlds(library, TEST_WORLD_DIR "/shapes.world");

  for (const auto &world : worlds)
  {
    StepWorld(world, 1000);

    auto model = world->GetModel(0);
    ASSERT_NE(nullptr, model);
    auto link = model->GetLink(0);
    ASSERT_NE(nullptr, link);
    auto frameData = link->FrameDataRelativeToWorld();
    EXPECT_EQ(ignition::math::Pose3d(0, 1.5, 0.5, 0, 0, 0),
              ignition::math::eigen3::convert(frameData.pose));
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

    EXPECT_EQ(1u, sphereLink->GetShapeCount());
    auto sphere2 = sphereLink->AttachSphereShape(
      "sphere2", 1.0, Eigen::Isometry3d::Identity());
    EXPECT_EQ(2u, sphereLink->GetShapeCount());
    EXPECT_EQ(sphere2, sphereLink->GetShape(1));

    auto box = world->GetModel("box");
    auto boxLink = box->GetLink(0);
    auto boxCollision = boxLink->GetShape(0);
    auto boxShape = boxCollision->CastToBoxShape();
    EXPECT_EQ(ignition::math::Vector3d(100, 100, 1),
              ignition::math::eigen3::convert(boxShape->GetSize()));

    auto box2 = boxLink->AttachBoxShape(
      "box2",
      ignition::math::eigen3::convert(
        ignition::math::Vector3d(1.2, 1.2, 1.2)),
      Eigen::Isometry3d::Identity());
    EXPECT_EQ(2u, boxLink->GetShapeCount());
    EXPECT_EQ(box2, boxLink->GetShape(1));

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
    auto boxAABB =
      boxCollision->GetAxisAlignedBoundingBox(*boxCollision);
    auto cylinderAABB =
      cylinderCollision->GetAxisAlignedBoundingBox(*cylinderCollision);

    EXPECT_EQ(ignition::math::Vector3d(-1, -1, -1),
              ignition::math::eigen3::convert(sphereAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(1, 1, 1),
              ignition::math::eigen3::convert(sphereAABB).Max());
    EXPECT_EQ(ignition::math::Vector3d(-50, -50, -0.5),
              ignition::math::eigen3::convert(boxAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(50, 50, 0.5),
              ignition::math::eigen3::convert(boxAABB).Max());
    EXPECT_EQ(ignition::math::Vector3d(-0.5, -0.5, -0.55),
              ignition::math::eigen3::convert(cylinderAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(0.5, 0.5, 0.55),
              ignition::math::eigen3::convert(cylinderAABB).Max());

    // check model AABB. By default, the AABBs are in world frame
    auto sphereModelAABB = sphere->GetAxisAlignedBoundingBox();
    auto boxModelAABB = box->GetAxisAlignedBoundingBox();
    auto cylinderModelAABB = cylinder->GetAxisAlignedBoundingBox();
    EXPECT_EQ(ignition::math::Vector3d(-1, 0.5, -0.5),
              ignition::math::eigen3::convert(sphereModelAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(1, 2.5, 1.5),
              ignition::math::eigen3::convert(sphereModelAABB).Max());
    EXPECT_EQ(ignition::math::Vector3d(-50, -50, -0.1),
              ignition::math::eigen3::convert(boxModelAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(50, 50, 1.1),
              ignition::math::eigen3::convert(boxModelAABB).Max());
    EXPECT_EQ(ignition::math::Vector3d(-3, -4.5, -1.5),
              ignition::math::eigen3::convert(cylinderModelAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(3, 1.5, 2.5),
              ignition::math::eigen3::convert(cylinderModelAABB).Max());
  }
}

TEST_P(SimulationFeatures_TEST, FreeGroup)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  auto worlds = LoadWorlds(library, TEST_WORLD_DIR "/shapes.world");

  for (const auto &world : worlds)
  {
    // model free group test
    auto model = world->GetModel("sphere");
    auto freeGroup = model->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);
    ASSERT_NE(nullptr, freeGroup->CanonicalLink());

    auto link = model->GetLink("sphere_link");
    auto freeGroupLink = link->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroupLink);

    freeGroup->SetWorldPose(
      ignition::math::eigen3::convert(
        ignition::math::Pose3d(0, 0, 2, 0, 0, 0)));
    freeGroup->SetWorldLinearVelocity(
      ignition::math::eigen3::convert(ignition::math::Vector3d(0.5, 0, 0.1)));
    freeGroup->SetWorldAngularVelocity(
      ignition::math::eigen3::convert(ignition::math::Vector3d(0.1, 0.2, 0)));

    auto frameData = model->GetLink(0)->FrameDataRelativeToWorld();
    EXPECT_EQ(ignition::math::Pose3d(0, 0, 2, 0, 0, 0),
              ignition::math::eigen3::convert(frameData.pose));
  }
}

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
    // Only box_colliding should collide with box_base
    EXPECT_EQ(1u, contacts.size());

    // Now disable collisions for the colliding box as well
    auto collidingShape = collidingBox->GetLink(0)->GetShape(0);
    auto filteredShape = filteredBox->GetLink(0)->GetShape(0);
    collidingShape->SetCollisionFilterMask(0xF0);
    // Also test the getter
    EXPECT_EQ(0xF0, collidingShape->GetCollisionFilterMask());
    // Step and make sure there are no collisions
    StepWorld(world);
    contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(0u, contacts.size());

    // Now remove both filter masks (no collisions will be filtered)
    // Equivalent to 0xFF
    collidingShape->RemoveCollisionFilterMask();
    filteredShape->RemoveCollisionFilterMask();
    StepWorld(world);
    // Expect box_filtered and box_colliding to collide with box_base
    contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(2u, contacts.size());
  }
}

TEST_P(SimulationFeatures_TEST, RetrieveContacts)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  auto worlds = LoadWorlds(library, TEST_WORLD_DIR "/shapes.world");

  for (const auto &world : worlds)
  {
    auto sphere = world->GetModel("sphere");
    auto sphereFreeGroup = sphere->FindFreeGroup();
    EXPECT_NE(nullptr, sphereFreeGroup);

    auto cylinder = world->GetModel("cylinder");
    auto cylinderFreeGroup = cylinder->FindFreeGroup();
    EXPECT_NE(nullptr, cylinderFreeGroup);

    auto box = world->GetModel("box");

    // step and get contacts
    StepWorld(world, 1);
    auto contacts = world->GetContactsFromLastStep();

    // large box in the middle should be intersecting with sphere and cylinder
    EXPECT_EQ(2u, contacts.size());
    unsigned int contactBoxSphere = 0u;
    unsigned int contactBoxCylinder = 0u;

    for (auto &contact : contacts)
    {
      const auto &contactPoint = contact.Get<ContactPoint>();
      ASSERT_TRUE(contactPoint.collision1);
      ASSERT_TRUE(contactPoint.collision2);
      EXPECT_NE(contactPoint.collision1, contactPoint.collision2);

      auto c1 = contactPoint.collision1;
      auto c2 = contactPoint.collision2;
      auto m1 = c1->GetLink()->GetModel();
      auto m2 = c2->GetLink()->GetModel();
      if ((m1->GetName() == "sphere" && m2->GetName() == "box") ||
          (m1->GetName() == "box" && m2->GetName() == "sphere"))
      {
        contactBoxSphere++;
        Eigen::Vector3d expectedContactPos = Eigen::Vector3d(0, 1.5, 0.5);
        EXPECT_TRUE(ignition::physics::test::Equal(expectedContactPos,
            contactPoint.point, 1e-6));
      }
      else if ((m1->GetName() == "box" && m2->GetName() == "cylinder") ||
          (m1->GetName() == "cylinder" && m2->GetName() == "box"))
      {
        contactBoxCylinder++;
        Eigen::Vector3d expectedContactPos = Eigen::Vector3d(0, -1.5, 0.5);
        EXPECT_TRUE(ignition::physics::test::Equal(expectedContactPos,
            contactPoint.point, 1e-6));
      }
      else
      {
        FAIL() << "There should not be contacts between: "
               << m1->GetName() << " " << m2->GetName();
      }
    }
    EXPECT_EQ(1u, contactBoxSphere);
    EXPECT_EQ(1u, contactBoxCylinder);

    // move sphere away
    sphereFreeGroup->SetWorldPose(ignition::math::eigen3::convert(
        ignition::math::Pose3d(0, 100, 0.5, 0, 0, 0)));

    // step and get contacts
    StepWorld(world, 1);
    contacts = world->GetContactsFromLastStep();

    // large box in the middle should be intersecting with cylinder
    EXPECT_EQ(1u, contacts.size());

    contactBoxCylinder = 0u;
    for (auto contact : contacts)
    {
      const auto &contactPoint = contact.Get<::ContactPoint>();
      ASSERT_TRUE(contactPoint.collision1);
      ASSERT_TRUE(contactPoint.collision2);
      EXPECT_NE(contactPoint.collision1, contactPoint.collision2);

      auto c1 = contactPoint.collision1;
      auto c2 = contactPoint.collision2;
      auto m1 = c1->GetLink()->GetModel();
      auto m2 = c2->GetLink()->GetModel();
      if ((m1->GetName() == "box" && m2->GetName() == "cylinder") ||
          (m1->GetName() == "cylinder" && m2->GetName() == "box"))
        contactBoxCylinder++;
      else
        FAIL() << "There should only be contacts between box and cylinder";

      Eigen::Vector3d expectedContactPos = Eigen::Vector3d(0, -1.5, 0.5);
      EXPECT_TRUE(ignition::physics::test::Equal(expectedContactPos,
          contactPoint.point, 1e-6));
    }
    EXPECT_EQ(1u, contactBoxCylinder);

    // move cylinder away
    cylinderFreeGroup->SetWorldPose(ignition::math::eigen3::convert(
        ignition::math::Pose3d(0, -100, 0.5, 0, 0, 0)));

    // step and get contacts
    StepWorld(world, 1);
    contacts = world->GetContactsFromLastStep();

    // no entities should be colliding
    EXPECT_TRUE(contacts.empty());
  }
}

INSTANTIATE_TEST_SUITE_P(
    PhysicsPlugins,
    SimulationFeatures_TEST,
    ::testing::ValuesIn(ignition::physics::test::g_PhysicsPluginLibraries)); // NOLINT

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
