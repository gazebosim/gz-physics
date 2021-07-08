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

#include <ignition/utils/SuppressWarning.hh>
#include <ignition/plugin/Loader.hh>

// Features
#include <ignition/physics/FindFeatures.hh>
#include <ignition/physics/GetBoundingBox.hh>
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/RequestEngine.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructNestedModel.hh>
#include <ignition/physics/sdf/ConstructLink.hh>
#include <ignition/physics/sdf/ConstructCollision.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Model.hh>
#include <sdf/Link.hh>

#include <test/PhysicsPluginsList.hh>
#include <test/Utils.hh>

#include "EntityManagementFeatures.hh"
#include "FreeGroupFeatures.hh"
#include "ShapeFeatures.hh"
#include "SimulationFeatures.hh"
#include "World.hh"

struct TestFeatureList : ignition::physics::FeatureList<
  ignition::physics::tpeplugin::SimulationFeatureList,
  ignition::physics::tpeplugin::ShapeFeatureList,
  ignition::physics::tpeplugin::EntityManagementFeatureList,
  ignition::physics::tpeplugin::FreeGroupFeatureList,
  ignition::physics::tpeplugin::RetrieveWorld,
  ignition::physics::GetContactsFromLastStepFeature,
  ignition::physics::LinkFrameSemantics,
  ignition::physics::GetModelBoundingBox,
  ignition::physics::sdf::ConstructSdfWorld,
  ignition::physics::sdf::ConstructSdfModel,
  ignition::physics::sdf::ConstructSdfNestedModel,
  ignition::physics::sdf::ConstructSdfLink,
  ignition::physics::sdf::ConstructSdfCollision
> { };

using TestEnginePtr = ignition::physics::Engine3dPtr<TestFeatureList>;
using TestWorldPtr = ignition::physics::World3dPtr<TestFeatureList>;
using TestModelPtr = ignition::physics::Model3dPtr<TestFeatureList>;
using TestLinkPtr = ignition::physics::Link3dPtr<TestFeatureList>;
using TestShapePtr = ignition::physics::Shape3dPtr<TestFeatureList>;
using ContactPoint = ignition::physics::World3d<TestFeatureList>::ContactPoint;

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

/////////////////////////////////////////////////
static ignition::math::Pose3d ResolveSdfPose(
    const ::sdf::SemanticPose &_semPose)
{
  ignition::math::Pose3d pose;
  ::sdf::Errors errors = _semPose.Resolve(pose);
  EXPECT_TRUE(errors.empty()) << errors;
  return pose;
}

std::unordered_set<TestWorldPtr> LoadWorldsPiecemeal(
    const std::string &_library, const std::string &_world)
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

    igndbg << " -- Plugin name: " << name << std::endl;

    auto engine =
      ignition::physics::RequestEngine3d<TestFeatureList>::From(plugin);
    EXPECT_NE(nullptr, engine);

    sdf::Root root;
    const sdf::Errors &errors = root.Load(_world);
    EXPECT_EQ(0u, errors.size()) << errors;

    EXPECT_EQ(1u, root.WorldCount());
    const sdf::World *sdfWorld = root.WorldByIndex(0);
    EXPECT_NE(nullptr, sdfWorld);

    sdf::World newWorld;
    newWorld.SetName(sdfWorld->Name());
    newWorld.SetGravity(sdfWorld->Gravity());
    auto world = engine->ConstructWorld(newWorld);
    if (nullptr == world)
      continue;

    std::unordered_map<const sdf::Model *, TestModelPtr> modelMap;
    std::unordered_map<const sdf::Link *, TestLinkPtr> linkMap;

    auto createModel = [&](const sdf::Model *_model,
        const sdf::Model *_parentModel = nullptr) {
      sdf::Model newSdfModel;
      newSdfModel.SetName(_model->Name());
      newSdfModel.SetRawPose(ResolveSdfPose(_model->SemanticPose()));
      newSdfModel.SetStatic(_model->Static());
      newSdfModel.SetSelfCollide(_model->SelfCollide());

      TestModelPtr newModel;
      if (nullptr != _parentModel)
      {
        auto it = modelMap.find(_parentModel);
        ASSERT_TRUE(it != modelMap.end());
        newModel = it->second->ConstructNestedModel(newSdfModel);
      }
      else
      {
        newModel = world->ConstructModel(newSdfModel);
      }

      EXPECT_NE(nullptr, newModel);
      if (nullptr != newModel)
      {
        modelMap[_model] = newModel;
      }
    };

    for (uint64_t i = 0; i < sdfWorld->ModelCount(); ++i)
    {
      const auto *model = sdfWorld->ModelByIndex(i);
      createModel(model);
      for (uint64_t nestedInd = 0; nestedInd < model->ModelCount(); ++nestedInd)
      {
        createModel(model->ModelByIndex(nestedInd), model);
      }
    }

    for (auto [sdfModel, physModel] : modelMap)
    {
      for (uint64_t li = 0; li < sdfModel->LinkCount(); ++li)
      {
        const auto *link = sdfModel->LinkByIndex(li);
        sdf::Link newSdfLink;
        newSdfLink.SetName(link->Name());
        newSdfLink.SetRawPose(ResolveSdfPose(link->SemanticPose()));
        newSdfLink.SetInertial(link->Inertial());

        auto newLink = physModel->ConstructLink(newSdfLink);
        EXPECT_NE(nullptr, newLink);
        if (nullptr == newLink)
          return worlds;

        linkMap[link] = newLink;
      }
    }

    for (auto [sdfLink, physLink] : linkMap)
    {
      for (uint64_t ci = 0; ci < sdfLink->CollisionCount(); ++ci)
      {
        physLink->ConstructCollision(*sdfLink->CollisionByIndex(ci));
      }
    }
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
  ignition::physics::ForwardStep::Input input;
  ignition::physics::ForwardStep::State state;
  ignition::physics::ForwardStep::Output output;

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
          output.Get<ignition::physics::ChangedWorldPoses>().entries.empty());
      checkedOutput = true;
    }
  }

  return checkedOutput;
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
    auto checkedOutput = StepWorld(world, true, 1000);
    EXPECT_TRUE(checkedOutput);

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

    auto ellipsoid = world->GetModel("ellipsoid");
    auto ellipsoidLink = ellipsoid->GetLink(0);
    auto ellipsoidCollision = ellipsoidLink->GetShape(0);
    auto ellipsoidShape = ellipsoidCollision->CastToEllipsoidShape();
    EXPECT_EQ(
      ignition::math::Vector3d(0.2, 0.3, 0.5),
      ignition::math::eigen3::convert(ellipsoidShape->GetRadii()));

    auto ellipsoid2 = ellipsoidLink->AttachEllipsoidShape(
      "ellipsoid2",
      ignition::math::eigen3::convert(ignition::math::Vector3d(0.2, 0.3, 0.5)),
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
    EXPECT_EQ(ignition::math::Vector3d(-0.2, -0.3, -0.5),
              ignition::math::eigen3::convert(ellipsoidAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(0.2, 0.3, 0.5),
              ignition::math::eigen3::convert(ellipsoidAABB).Max());
    EXPECT_EQ(ignition::math::Vector3d(-0.2, -0.2, -0.5),
              ignition::math::eigen3::convert(capsuleAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(0.2, 0.2, 0.5),
              ignition::math::eigen3::convert(capsuleAABB).Max());

    // check model AABB. By default, the AABBs are in world frame
    auto sphereModelAABB = sphere->GetAxisAlignedBoundingBox();
    auto boxModelAABB = box->GetAxisAlignedBoundingBox();
    auto cylinderModelAABB = cylinder->GetAxisAlignedBoundingBox();
    auto ellipsoidModelAABB = ellipsoid->GetAxisAlignedBoundingBox();
    auto capsuleModelAABB = capsule->GetAxisAlignedBoundingBox();
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
    EXPECT_EQ(ignition::math::Vector3d(-0.2, -5.3, 0.2),
              ignition::math::eigen3::convert(ellipsoidModelAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(0.2, -4.7, 1.2),
              ignition::math::eigen3::convert(ellipsoidModelAABB).Max());
    EXPECT_EQ(ignition::math::Vector3d(-0.2, -3.2, 0),
              ignition::math::eigen3::convert(capsuleModelAABB).Min());
    EXPECT_EQ(ignition::math::Vector3d(0.2, -2.8, 1),
              ignition::math::eigen3::convert(capsuleModelAABB).Max());
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
    IGN_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
    ASSERT_NE(nullptr, freeGroup->CanonicalLink());
    IGN_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
    ASSERT_NE(nullptr, freeGroup->RootLink());

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

TEST_P(SimulationFeatures_TEST, NestedFreeGroup)
{
  const std::string library = GetParam();
  if (library.empty())
    return;

  auto worlds =
      LoadWorldsPiecemeal(library, TEST_WORLD_DIR "/nested_model.world");

  for (const auto &world : worlds)
  {
    auto tpeWorld = world->GetTpeLibWorld();
    ASSERT_NE(nullptr, tpeWorld);

    // model free group test
    auto model = world->GetModel("model");
    ASSERT_NE(nullptr, model);
    auto freeGroup = model->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);
    ASSERT_NE(nullptr, freeGroup->RootLink());

    ignition::math::Pose3d newPose(1, 1, 0, 0, 0, 0);
    freeGroup->SetWorldPose(ignition::math::eigen3::convert(newPose));

    {
      auto link = model->GetLink("link");
      ASSERT_NE(nullptr, link);
      auto frameData = link->FrameDataRelativeToWorld();
      EXPECT_EQ(newPose,
          ignition::math::eigen3::convert(frameData.pose));
    }
    {
      auto nestedModel = model->GetNestedModel("nested_model");
      ASSERT_NE(nullptr, nestedModel);
      auto nestedLink = nestedModel->GetLink("nested_link");
      ASSERT_NE(nullptr, nestedLink);

      // Poses from SDF
      ignition::math::Pose3d nestedModelPose(1, 2, 2, 0, 0, 0);
      ignition::math::Pose3d nestedLinkPose(3, 1, 1, 0, 0, IGN_PI_2);

      ignition::math::Pose3d nestedLinkExpectedPose =
        newPose * nestedModelPose * nestedLinkPose;

      EXPECT_EQ(nestedLinkExpectedPose,
                ignition::math::eigen3::convert(
                    nestedLink->FrameDataRelativeToWorld().pose));
    }
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

    auto checkedOutput = StepWorld(world, true);
    EXPECT_TRUE(checkedOutput);
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
    EXPECT_EQ(4u, contacts.size());
    unsigned int contactBoxSphere = 0u;
    unsigned int contactBoxCylinder = 0u;
    unsigned int contactBoxCapsule = 0u;
    unsigned int contactBoxEllipsoid = 0u;

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
      else if ((m1->GetName() == "box" && m2->GetName() == "capsule") ||
          (m1->GetName() == "capsule" && m2->GetName() == "box"))
      {
        contactBoxCapsule++;
        Eigen::Vector3d expectedContactPos = Eigen::Vector3d(0.0, -3, 0.5);
        EXPECT_TRUE(ignition::physics::test::Equal(expectedContactPos,
            contactPoint.point, 1e-6));
      }
      else if ((m1->GetName() == "box" && m2->GetName() == "ellipsoid") ||
          (m1->GetName() == "ellipsoid" && m2->GetName() == "box"))
      {
        contactBoxEllipsoid++;
        Eigen::Vector3d expectedContactPos = Eigen::Vector3d(0.0, -5, 0.6);
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
    EXPECT_EQ(1u, contactBoxCapsule);
    EXPECT_EQ(1u, contactBoxEllipsoid);

    // move sphere away
    sphereFreeGroup->SetWorldPose(ignition::math::eigen3::convert(
        ignition::math::Pose3d(0, 100, 0.5, 0, 0, 0)));

    // step and get contacts
    checkedOutput = StepWorld(world, false);
    EXPECT_FALSE(checkedOutput);
    contacts = world->GetContactsFromLastStep();

    // large box in the middle should be intersecting with cylinder, capsule,
    // ellipsoid
    EXPECT_EQ(3u, contacts.size());

    contactBoxCylinder = 0u;
    contactBoxCapsule = 0u;
    contactBoxEllipsoid = 0u;
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
      {
        contactBoxCylinder++;
        Eigen::Vector3d expectedContactPos = Eigen::Vector3d(0, -1.5, 0.5);
        EXPECT_TRUE(ignition::physics::test::Equal(expectedContactPos,
            contactPoint.point, 1e-6));
      }
      else if ((m1->GetName() == "box" && m2->GetName() == "capsule") ||
          (m1->GetName() == "capsule" && m2->GetName() == "box"))
      {
        contactBoxCapsule++;
        Eigen::Vector3d expectedContactPos = Eigen::Vector3d(0.0, -3, 0.5);
        EXPECT_TRUE(ignition::physics::test::Equal(expectedContactPos,
            contactPoint.point, 1e-6));
      }
      else if ((m1->GetName() == "box" && m2->GetName() == "ellipsoid") ||
          (m1->GetName() == "ellipsoid" && m2->GetName() == "box"))
      {
        contactBoxEllipsoid++;
        Eigen::Vector3d expectedContactPos = Eigen::Vector3d(0.0, -5, 0.6);
        EXPECT_TRUE(ignition::physics::test::Equal(expectedContactPos,
            contactPoint.point, 1e-6));
      }
      else
      {
        FAIL() << "There should only be contacts between box and cylinder";
      }
    }
    EXPECT_EQ(1u, contactBoxCylinder);
    EXPECT_EQ(1u, contactBoxCapsule);
    EXPECT_EQ(1u, contactBoxEllipsoid);

    // move cylinder away
    cylinderFreeGroup->SetWorldPose(ignition::math::eigen3::convert(
        ignition::math::Pose3d(0, -100, 0.5, 0, 0, 0)));

    // move capsule away
    capsuleFreeGroup->SetWorldPose(ignition::math::eigen3::convert(
        ignition::math::Pose3d(0, -100, 100, 0, 0, 0)));

    // move ellipsoid away
    ellipsoidFreeGroup->SetWorldPose(ignition::math::eigen3::convert(
        ignition::math::Pose3d(0, -100, -100, 0, 0, 0)));

    // step and get contacts
    checkedOutput = StepWorld(world, false);
    EXPECT_FALSE(checkedOutput);
    contacts = world->GetContactsFromLastStep();

    // no entities should be colliding
    EXPECT_TRUE(contacts.empty());
  }
}

INSTANTIATE_TEST_CASE_P(PhysicsPlugins, SimulationFeatures_TEST,
  ::testing::ValuesIn(ignition::physics::test::g_PhysicsPluginLibraries),); // NOLINT

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
