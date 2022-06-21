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

#include <set>
#include <string>
#include <unordered_set>

#include <gz/common/Console.hh>

#include <gz/math/eigen3/Conversions.hh>

#include "../helpers/TestLibLoader.hh"

#include <gz/physics/sdf/ConstructJoint.hh>
#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructCollision.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include "gz/physics/BoxShape.hh"
#include <gz/physics/GetContacts.hh>
#include "gz/physics/CylinderShape.hh"
#include "gz/physics/CapsuleShape.hh"
#include "gz/physics/EllipsoidShape.hh"
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/GetBoundingBox.hh>
#include "gz/physics/SphereShape.hh"

#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>

#include <sdf/Root.hh>

// The features that an engine must have to be loaded by this loader.
using Features = gz::physics::FeatureList<
  gz::physics::ConstructEmptyWorldFeature,

  gz::physics::FindFreeGroupFeature,
  gz::physics::SetFreeGroupWorldPose,
  gz::physics::SetFreeGroupWorldVelocity,

  gz::physics::GetContactsFromLastStepFeature,
  gz::physics::CollisionFilterMaskFeature,

  gz::physics::GetModelFromWorld,
  gz::physics::GetLinkFromModel,
  gz::physics::GetShapeFromLink,
  gz::physics::GetModelBoundingBox,

  // gz::physics::sdf::ConstructSdfJoint,
  gz::physics::sdf::ConstructSdfLink,
  gz::physics::sdf::ConstructSdfModel,
  gz::physics::sdf::ConstructSdfCollision,
  gz::physics::sdf::ConstructSdfWorld,

  gz::physics::ForwardStep,

  gz::physics::AttachBoxShapeFeature,
  gz::physics::AttachSphereShapeFeature,
  gz::physics::AttachCylinderShapeFeature,
  gz::physics::AttachEllipsoidShapeFeature,
  gz::physics::AttachCapsuleShapeFeature,
  gz::physics::GetSphereShapeProperties,
  gz::physics::GetBoxShapeProperties,
  gz::physics::GetCylinderShapeProperties,
  gz::physics::GetCapsuleShapeProperties,
  gz::physics::GetEllipsoidShapeProperties
>;

using TestWorldPtr = gz::physics::World3dPtr<Features>;
using TestContactPoint = gz::physics::World3d<Features>::ContactPoint;

class SimulationFeaturesTest:
public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    loader.LoadLib(SimulationFeaturesTest::GetLibToTest());

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<Features>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in "
                << GetLibToTest() << std::endl;
      // TODO(ahcorde): If we update gtest we can use here GTEST_SKIP()
    }
  }
};

std::unordered_set<TestWorldPtr> LoadWorlds(
    const gz::plugin::Loader &_loader,
    const std::set<std::string> pluginNames,
    const std::string &_world)
{
  std::unordered_set<TestWorldPtr> worlds;
  for (const std::string &name : pluginNames)
  {
    gz::plugin::PluginPtr plugin = _loader.Instantiate(name);

    gzdbg << " -- Plugin name: " << name << std::endl;

    auto engine =
      gz::physics::RequestEngine3d<Features>::From(plugin);
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

/// \brief Step forward in a world
/// \param[in] _world The world to step in
/// \param[in] _firstTime Whether this is the very first time this world is
/// being stepped in (true) or not (false)
/// \param[in] _numSteps The number of steps to take in _world
/// \return true if the forward step output was checked, false otherwise
bool StepWorld(const TestWorldPtr &_world, bool _firstTime,
    const std::size_t _numSteps = 1)
{
  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;

  bool checkedOutput = false;
  for (size_t i = 0; i < _numSteps; ++i)
  {
    _world->Step(output, state, input);

    // If link poses have changed, this should have been written to output.
    // Link poses are considered "changed" if they are new, so if this is the
    // very first step in a world, all of the link data is new and output
    // should not be empty
    if (_firstTime && (i == 0))
    {
      EXPECT_FALSE(
          output.Get<gz::physics::ChangedWorldPoses>().entries.empty());
      checkedOutput = true;
    }
  }

  return checkedOutput;
}

/////////////////////////////////////////////////
TEST_F(SimulationFeaturesTest, StepWorld)
{
  auto worlds = LoadWorlds(loader, pluginNames, TEST_WORLD_DIR "/shapes.world");
  for (const auto &world : worlds)
  {
    auto checkedOutput = StepWorld(world, true, 1000);
    EXPECT_TRUE(checkedOutput);
  }
}

/////////////////////////////////////////////////
TEST_F(SimulationFeaturesTest, ShapeFeatures)
{
  auto worlds = LoadWorlds(loader, pluginNames, TEST_WORLD_DIR "/shapes.world");
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
    EXPECT_EQ(gz::math::Vector3d(100, 100, 1),
              gz::math::eigen3::convert(boxShape->GetSize()));

    auto box2 = boxLink->AttachBoxShape(
      "box2",
      gz::math::eigen3::convert(
        gz::math::Vector3d(1.2, 1.2, 1.2)),
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

    auto ellipsoid = world->GetModel("ellipsoid");
    auto ellipsoidLink = ellipsoid->GetLink(0);
    auto ellipsoidCollision = ellipsoidLink->GetShape(0);
    auto ellipsoidShape = ellipsoidCollision->CastToEllipsoidShape();
    EXPECT_TRUE(
      gz::math::Vector3d(0.2, 0.3, 0.5).Equal(
      gz::math::eigen3::convert(ellipsoidShape->GetRadii()), 0.1));

    auto ellipsoid2 = ellipsoidLink->AttachEllipsoidShape(
      "ellipsoid2",
      gz::math::eigen3::convert(gz::math::Vector3d(0.2, 0.3, 0.5)),
      Eigen::Isometry3d::Identity());
    EXPECT_EQ(2u, ellipsoidLink->GetShapeCount());
    EXPECT_EQ(ellipsoid2, ellipsoidLink->GetShape(1));

    auto capsule = world->GetModel("capsule");
    auto capsuleLink = capsule->GetLink(0);
    auto capsuleCollision = capsuleLink->GetShape(0);
    auto capsuleShape = capsuleCollision->CastToCapsuleShape();
    EXPECT_NEAR(0.2, capsuleShape->GetRadius(), 1e-6);
    EXPECT_NEAR(0.6, capsuleShape->GetLength(), 1e-6);

    auto capsule2 = capsuleLink->AttachCapsuleShape(
      "capsule2", 0.2, 0.6, Eigen::Isometry3d::Identity());
    EXPECT_EQ(2u, capsuleLink->GetShapeCount());
    EXPECT_EQ(capsule2, capsuleLink->GetShape(1));

    // Test the bounding boxes in the local frames
    auto sphereAABB =
      sphereCollision->GetAxisAlignedBoundingBox(*sphereCollision);
    auto boxAABB =
      boxCollision->GetAxisAlignedBoundingBox(*boxCollision);
    auto cylinderAABB =
      cylinderCollision->GetAxisAlignedBoundingBox(*cylinderCollision);
    auto ellipsoidAABB =
      ellipsoidCollision->GetAxisAlignedBoundingBox(*ellipsoidCollision);
    auto capsuleAABB =
      capsuleCollision->GetAxisAlignedBoundingBox(*capsuleCollision);

    EXPECT_TRUE(gz::math::Vector3d(-1, -1, -1).Equal(
                gz::math::eigen3::convert(sphereAABB).Min(), 0.1));
    EXPECT_EQ(gz::math::Vector3d(1, 1, 1),
              gz::math::eigen3::convert(sphereAABB).Max());
    EXPECT_EQ(gz::math::Vector3d(-50, -50, -0.5),
              gz::math::eigen3::convert(boxAABB).Min());
    EXPECT_EQ(gz::math::Vector3d(50, 50, 0.5),
              gz::math::eigen3::convert(boxAABB).Max());
    EXPECT_EQ(gz::math::Vector3d(-0.5, -0.5, -0.55),
              gz::math::eigen3::convert(cylinderAABB).Min());
    EXPECT_EQ(gz::math::Vector3d(0.5, 0.5, 0.55),
              gz::math::eigen3::convert(cylinderAABB).Max());
    EXPECT_TRUE(gz::math::Vector3d(-0.2, -0.3, -0.5).Equal(
                gz::math::eigen3::convert(ellipsoidAABB).Min(), 0.1));
    EXPECT_TRUE(gz::math::Vector3d(0.2, 0.3, 0.5).Equal(
                gz::math::eigen3::convert(ellipsoidAABB).Max(), 0.1));
    EXPECT_TRUE(gz::math::Vector3d(-0.2, -0.2, -0.5).Equal(
                gz::math::eigen3::convert(capsuleAABB).Min(), 0.1));
    EXPECT_TRUE(gz::math::Vector3d(0.2, 0.2, 0.5).Equal(
                gz::math::eigen3::convert(capsuleAABB).Max(), 0.1));

    // check model AABB. By default, the AABBs are in world frame
    auto sphereModelAABB = sphere->GetAxisAlignedBoundingBox();
    auto boxModelAABB = box->GetAxisAlignedBoundingBox();
    auto cylinderModelAABB = cylinder->GetAxisAlignedBoundingBox();
    auto ellipsoidModelAABB = ellipsoid->GetAxisAlignedBoundingBox();
    auto capsuleModelAABB = capsule->GetAxisAlignedBoundingBox();
    EXPECT_EQ(gz::math::Vector3d(-1, 0.5, -0.5),
              gz::math::eigen3::convert(sphereModelAABB).Min());
    EXPECT_EQ(gz::math::Vector3d(1, 2.5, 1.5),
              gz::math::eigen3::convert(sphereModelAABB).Max());
    EXPECT_EQ(gz::math::Vector3d(-50, -50, -0.1),
              gz::math::eigen3::convert(boxModelAABB).Min());
    EXPECT_EQ(gz::math::Vector3d(50, 50, 1.1),
              gz::math::eigen3::convert(boxModelAABB).Max());
    EXPECT_EQ(gz::math::Vector3d(-3, -4.5, -1.5),
              gz::math::eigen3::convert(cylinderModelAABB).Min());
    EXPECT_EQ(gz::math::Vector3d(3, 1.5, 2.5),
              gz::math::eigen3::convert(cylinderModelAABB).Max());
    EXPECT_TRUE(gz::math::Vector3d(-0.2, -5.3, 0.2).Equal(
                gz::math::eigen3::convert(ellipsoidModelAABB).Min(), 0.1));
    EXPECT_TRUE(gz::math::Vector3d(0.2, -4.7, 1.2).Equal(
                gz::math::eigen3::convert(ellipsoidModelAABB).Max(), 0.1));
    EXPECT_EQ(gz::math::Vector3d(-0.2, -3.2, 0),
              gz::math::eigen3::convert(capsuleModelAABB).Min());
    EXPECT_EQ(gz::math::Vector3d(0.2, -2.8, 1),
              gz::math::eigen3::convert(capsuleModelAABB).Max());
  }
}

TEST_F(SimulationFeaturesTest, FreeGroup)
{
  auto worlds = LoadWorlds(loader, pluginNames, TEST_WORLD_DIR "/shapes.world");

  for (const auto &world : worlds)
  {
    // model free group test
    auto model = world->GetModel("sphere");
    auto freeGroup = model->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);
    GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
    ASSERT_NE(nullptr, freeGroup->CanonicalLink());
    GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
    ASSERT_NE(nullptr, freeGroup->RootLink());

    auto link = model->GetLink("sphere_link");
    auto freeGroupLink = link->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroupLink);

    StepWorld(world, true);

    freeGroup->SetWorldPose(
      gz::math::eigen3::convert(
        gz::math::Pose3d(0, 0, 2, 0, 0, 0)));
    freeGroup->SetWorldLinearVelocity(
      gz::math::eigen3::convert(gz::math::Vector3d(0.1, 0.2, 0.3)));
    freeGroup->SetWorldAngularVelocity(
      gz::math::eigen3::convert(gz::math::Vector3d(0.4, 0.5, 0.6)));

    auto frameData = model->GetLink(0)->FrameDataRelativeToWorld();
    EXPECT_EQ(gz::math::Pose3d(0, 0, 2, 0, 0, 0),
              gz::math::eigen3::convert(frameData.pose));

    // Step the world
    StepWorld(world, false);
    // Check that the first link's velocities are updated
    frameData = model->GetLink(0)->FrameDataRelativeToWorld();
    EXPECT_TRUE(gz::math::Vector3d(0.1, 0.2, 0.3).Equal(
                gz::math::eigen3::convert(frameData.linearVelocity), 0.1));
    EXPECT_EQ(gz::math::Vector3d(0.4, 0.5, 0.6),
              gz::math::eigen3::convert(frameData.angularVelocity));
  }
}


TEST_F(SimulationFeaturesTest, CollideBitmasks)
{
  auto worlds = LoadWorlds(loader, pluginNames, TEST_WORLD_DIR "/shapes_bitmask.sdf");

  for (const auto &world : worlds)
  {
    auto baseBox = world->GetModel("box_base");
    auto filteredBox = world->GetModel("box_filtered");
    auto collidingBox = world->GetModel("box_colliding");

    auto checkedOutput = StepWorld(world, true);
    EXPECT_TRUE(checkedOutput);
    auto contacts = world->GetContactsFromLastStep();
    // Only box_colliding should collide with box_base
    EXPECT_NE(0u, contacts.size());

    // Now disable collisions for the colliding box as well
    auto collidingShape = collidingBox->GetLink(0)->GetShape(0);
    auto filteredShape = filteredBox->GetLink(0)->GetShape(0);
    collidingShape->SetCollisionFilterMask(0xF0);
    // Also test the getter
    EXPECT_EQ(0xF0, collidingShape->GetCollisionFilterMask());
    // Step and make sure there are no collisions
    checkedOutput = StepWorld(world, false);
    EXPECT_FALSE(checkedOutput);
    contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(0u, contacts.size());

    // Now remove both filter masks (no collisions will be filtered)
    // Equivalent to 0xFF
    collidingShape->RemoveCollisionFilterMask();
    filteredShape->RemoveCollisionFilterMask();
    checkedOutput = StepWorld(world, false);
    EXPECT_FALSE(checkedOutput);
    // Expect box_filtered and box_colliding to collide with box_base
    contacts = world->GetContactsFromLastStep();
    EXPECT_NE(0u, contacts.size());
  }
}


TEST_F(SimulationFeaturesTest, RetrieveContacts)
{
  auto worlds = LoadWorlds(loader, pluginNames, TEST_WORLD_DIR "/shapes.world");

  for (const auto &world : worlds)
  {
    auto sphere = world->GetModel("sphere");
    auto sphereFreeGroup = sphere->FindFreeGroup();
    EXPECT_NE(nullptr, sphereFreeGroup);

    auto cylinder = world->GetModel("cylinder");
    auto cylinderFreeGroup = cylinder->FindFreeGroup();
    EXPECT_NE(nullptr, cylinderFreeGroup);

    auto capsule = world->GetModel("capsule");
    auto capsuleFreeGroup = capsule->FindFreeGroup();
    EXPECT_NE(nullptr, capsuleFreeGroup);

    auto ellipsoid = world->GetModel("ellipsoid");
    auto ellipsoidFreeGroup = ellipsoid->FindFreeGroup();
    EXPECT_NE(nullptr, ellipsoidFreeGroup);

    auto box = world->GetModel("box");

    // step and get contacts
    auto checkedOutput = StepWorld(world, true);
    EXPECT_TRUE(checkedOutput);
    auto contacts = world->GetContactsFromLastStep();

    // large box in the middle should be intersecting with sphere, cylinder,
    // capsule and ellipsoid
    EXPECT_NE(0u, contacts.size());
    unsigned int contactBoxSphere = 0u;
    unsigned int contactBoxCylinder = 0u;
    unsigned int contactBoxCapsule = 0u;
    unsigned int contactBoxEllipsoid = 0u;

    for (auto &contact : contacts)
    {
      const auto &contactPoint = contact.Get<TestContactPoint>();
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
      }
      else if ((m1->GetName() == "box" && m2->GetName() == "cylinder") ||
          (m1->GetName() == "cylinder" && m2->GetName() == "box"))
      {
        contactBoxCylinder++;
      }
      else if ((m1->GetName() == "box" && m2->GetName() == "capsule") ||
          (m1->GetName() == "capsule" && m2->GetName() == "box"))
      {
        contactBoxCapsule++;
      }
      else if ((m1->GetName() == "box" && m2->GetName() == "ellipsoid") ||
          (m1->GetName() == "ellipsoid" && m2->GetName() == "box"))
      {
        contactBoxEllipsoid++;
      }
      else
      {
        FAIL() << "There should not be contacts between: "
               << m1->GetName() << " " << m2->GetName();
      }
    }
    EXPECT_NE(0u, contactBoxSphere);
    EXPECT_NE(0u, contactBoxCylinder);
    EXPECT_NE(0u, contactBoxCapsule);
    EXPECT_NE(0u, contactBoxEllipsoid);

    // move sphere away
    sphereFreeGroup->SetWorldPose(gz::math::eigen3::convert(
        gz::math::Pose3d(0, 100, 0.5, 0, 0, 0)));

    // step and get contacts
    checkedOutput = StepWorld(world, false);
    EXPECT_FALSE(checkedOutput);
    contacts = world->GetContactsFromLastStep();

    // large box in the middle should be intersecting with cylinder, capsule,
    // ellipsoid
    EXPECT_NE(0u, contacts.size());

    contactBoxCylinder = 0u;
    contactBoxCapsule = 0u;
    contactBoxEllipsoid = 0u;
    for (auto contact : contacts)
    {
      const auto &contactPoint = contact.Get<::TestContactPoint>();
      ASSERT_TRUE(contactPoint.collision1);
      ASSERT_TRUE(contactPoint.collision2);
      EXPECT_NE(contactPoint.collision1, contactPoint.collision2);

      auto c1 = contactPoint.collision1;
      auto c2 = contactPoint.collision2;
      auto m1 = c1->GetLink()->GetModel();
      auto m2 = c2->GetLink()->GetModel();
      if ((m1->GetName() == "box" && m2->GetName() == "cylinder") ||
          (m1->GetName() == "cylinder" && m2->GetName() == "box"))
      {
        contactBoxCylinder++;
      }
      else if ((m1->GetName() == "box" && m2->GetName() == "capsule") ||
          (m1->GetName() == "capsule" && m2->GetName() == "box"))
      {
        contactBoxCapsule++;
      }
      else if ((m1->GetName() == "box" && m2->GetName() == "ellipsoid") ||
          (m1->GetName() == "ellipsoid" && m2->GetName() == "box"))
      {
        contactBoxEllipsoid++;
      }
      else
      {
        FAIL() << "There should only be contacts between box and cylinder";
      }
    }
    EXPECT_NE(0u, contactBoxCylinder);
    EXPECT_NE(0u, contactBoxCapsule);
    EXPECT_NE(0u, contactBoxEllipsoid);

    // move cylinder away
    cylinderFreeGroup->SetWorldPose(gz::math::eigen3::convert(
        gz::math::Pose3d(0, -100, 0.5, 0, 0, 0)));

    // move capsule away
    capsuleFreeGroup->SetWorldPose(gz::math::eigen3::convert(
        gz::math::Pose3d(0, -100, 100, 0, 0, 0)));

    // move ellipsoid away
    ellipsoidFreeGroup->SetWorldPose(gz::math::eigen3::convert(
        gz::math::Pose3d(0, -100, -100, 0, 0, 0)));

    // step and get contacts
    checkedOutput = StepWorld(world, false);
    EXPECT_FALSE(checkedOutput);
    contacts = world->GetContactsFromLastStep();

    // no entities should be colliding
    EXPECT_TRUE(contacts.empty());
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  SimulationFeaturesTest::init(argc, argv);
  return RUN_ALL_TESTS();
}
