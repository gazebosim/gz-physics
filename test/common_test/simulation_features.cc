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
#include <gz/plugin/Loader.hh>

#include <gz/math/eigen3/Conversions.hh>

#include "test/TestLibLoader.hh"
#include "test/Utils.hh"
#include "Worlds.hh"

#include <gz/physics/sdf/ConstructJoint.hh>
#include <gz/physics/sdf/ConstructLink.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructCollision.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include "gz/physics/BoxShape.hh"
#include "gz/physics/ContactProperties.hh"
#include "gz/physics/CapsuleShape.hh"
#include "gz/physics/ConeShape.hh"
#include "gz/physics/CylinderShape.hh"
#include "gz/physics/EllipsoidShape.hh"
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/GetBoundingBox.hh>
#include <gz/physics/GetContacts.hh>
#include "gz/physics/SphereShape.hh"

#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/FindFeatures.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/World.hh>

#include <sdf/Root.hh>

// The features that an engine must have to be loaded by this loader.
struct Features : gz::physics::FeatureList<
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

  gz::physics::sdf::ConstructSdfLink,
  gz::physics::sdf::ConstructSdfModel,
  gz::physics::sdf::ConstructSdfCollision,
  gz::physics::sdf::ConstructSdfWorld,

  gz::physics::ForwardStep,

  gz::physics::AttachBoxShapeFeature,
  gz::physics::AttachSphereShapeFeature,
  gz::physics::AttachConeShapeFeature,
  gz::physics::AttachCylinderShapeFeature,
  gz::physics::AttachEllipsoidShapeFeature,
  gz::physics::AttachCapsuleShapeFeature,
  gz::physics::GetSphereShapeProperties,
  gz::physics::GetBoxShapeProperties,
  gz::physics::GetConeShapeProperties,
  gz::physics::GetCylinderShapeProperties,
  gz::physics::GetCapsuleShapeProperties,
  gz::physics::GetEllipsoidShapeProperties
> {};

template <class T>
class SimulationFeaturesTest:
  public testing::Test, public gz::physics::TestLibLoader
{
  // Documentation inherited
  public: void SetUp() override
  {
    gz::common::Console::SetVerbosity(4);

    std::unordered_set<std::string> sharedLibs =
        loader.LoadLib(SimulationFeaturesTest::GetLibToTest());

    ASSERT_FALSE(sharedLibs.empty())
        << "Failed to load plugin library " << GetLibToTest();

    // TODO(ahcorde): We should also run the 3f, 2d, and 2f variants of
    // FindFeatures
    pluginNames = gz::physics::FindFeatures3d<T>::From(loader);
    if (pluginNames.empty())
    {
      std::cerr << "No plugins with required features found in "
                << GetLibToTest() << std::endl;
      GTEST_SKIP();
    }
  }

  public: std::set<std::string> pluginNames;
  public: gz::plugin::Loader loader;
};

template <class T>
gz::physics::World3dPtr<T> LoadPluginAndWorld(
    const gz::plugin::Loader &_loader,
    const std::string &_pluginName,
    const std::string &_world)
{
  gz::plugin::PluginPtr plugin = _loader.Instantiate(_pluginName);

  gzdbg << " -- Plugin name: " << _pluginName << std::endl;

  auto engine =
    gz::physics::RequestEngine3d<T>::From(plugin);
  EXPECT_NE(nullptr, engine);

  sdf::Root root;
  const sdf::Errors &errors = root.Load(_world);
  EXPECT_EQ(0u, errors.size());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  auto world = engine->ConstructWorld(*sdfWorld);
  EXPECT_NE(nullptr, world);
  return world;
}

using AssertVectorApprox = gz::physics::test::AssertVectorApprox;

/// \brief Step forward in a world
/// \param[in] _world The world to step in
/// \param[in] _firstTime Whether this is the very first time this world is
/// being stepped in (true) or not (false)
/// \param[in] _numSteps The number of steps to take in _world
/// \return true if the forward step output was checked, false otherwise
template <class T>
std::pair<bool, gz::physics::ForwardStep::Output> StepWorld(
  const gz::physics::World3dPtr<T> &_world,
  bool _firstTime,
  const std::size_t _numSteps = 1)
{
  EXPECT_NE(nullptr, _world);
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

    if (i == _numSteps - 2)
    {
      // This is needed so Step populates WorldPoses even though we haven't
      // queried them.
      output.ResetQueries();
    }
  }

  return std::make_pair(checkedOutput, output);
}

// The features that an engine must have to be loaded by this loader.
struct FeaturesContacts : gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::GetContactsFromLastStepFeature,
  gz::physics::ForwardStep
> {};

template <class T>
class SimulationFeaturesContactsTest :
  public SimulationFeaturesTest<T>{};
using SimulationFeaturesContactsTestTypes =
  ::testing::Types<FeaturesContacts>;
TYPED_TEST_SUITE(SimulationFeaturesContactsTest,
                 SimulationFeaturesContactsTestTypes);

/////////////////////////////////////////////////
TYPED_TEST(SimulationFeaturesContactsTest, Contacts)
{
  for (const std::string &name : this->pluginNames)
  {
    auto world = LoadPluginAndWorld<FeaturesContacts>(
      this->loader,
      name,
      common_test::worlds::kShapesWorld);
    auto checkedOutput = StepWorld<FeaturesContacts>(world, true, 1).first;
    EXPECT_TRUE(checkedOutput);

    auto contacts = world->GetContactsFromLastStep();
    // Large box collides with other shapes
    EXPECT_NE(0u, contacts.size());
  }
}

// The features that an engine must have to be loaded by this loader.
struct FeaturesCollisionPairMaxContacts : gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::GetContactsFromLastStepFeature,
  gz::physics::ForwardStep,
  gz::physics::CollisionPairMaxContacts
> {};

template <class T>
class SimulationFeaturesCollisionPairMaxContactsTest :
  public SimulationFeaturesTest<T>{};
using SimulationFeaturesCollisionPairMaxContactsTestTypes =
  ::testing::Types<FeaturesCollisionPairMaxContacts>;
TYPED_TEST_SUITE(SimulationFeaturesCollisionPairMaxContactsTest,
                 SimulationFeaturesCollisionPairMaxContactsTestTypes);

/////////////////////////////////////////////////
TYPED_TEST(SimulationFeaturesCollisionPairMaxContactsTest,
    CollisionPairMaxContacts)
{
  for (const std::string &name : this->pluginNames)
  {
    auto world = LoadPluginAndWorld<FeaturesCollisionPairMaxContacts>(
      this->loader,
      name,
      common_test::worlds::kShapesWorld);
    auto checkedOutput = StepWorld<FeaturesCollisionPairMaxContacts>(
        world, true, 1).first;
    EXPECT_TRUE(checkedOutput);

    auto contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(std::numeric_limits<std::size_t>::max(),
              world->GetCollisionPairMaxContacts());
    // Large box collides with other shapes
    EXPECT_GT(contacts.size(), 30u);

    world->SetCollisionPairMaxContacts(1u);
    EXPECT_EQ(1u, world->GetCollisionPairMaxContacts());
    checkedOutput = StepWorld<FeaturesCollisionPairMaxContacts>(
        world, true, 1).first;
    EXPECT_TRUE(checkedOutput);

    contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(5u, contacts.size());

    world->SetCollisionPairMaxContacts(0u);
    EXPECT_EQ(0u, world->GetCollisionPairMaxContacts());
    checkedOutput = StepWorld<FeaturesCollisionPairMaxContacts>(
        world, true, 1).first;
    EXPECT_TRUE(checkedOutput);

    contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(0u, contacts.size());
  }
}

// The features that an engine must have to be loaded by this loader.
struct FeaturesStep : gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::ForwardStep
> {};

template <class T>
class SimulationFeaturesStepTest :
  public SimulationFeaturesTest<T>{};
using SimulationFeaturesStepTestTypes =
  ::testing::Types<FeaturesStep>;
TYPED_TEST_SUITE(SimulationFeaturesStepTest,
                 SimulationFeaturesStepTestTypes);

/////////////////////////////////////////////////
TYPED_TEST(SimulationFeaturesStepTest, StepWorld)
{
  for (const std::string &name : this->pluginNames)
  {
#ifdef _WIN32
    // See https://github.com/gazebosim/gz-physics/issues/483
    CHECK_UNSUPPORTED_ENGINE(name, "bullet")
#endif
    auto world = LoadPluginAndWorld<FeaturesStep>(
      this->loader,
      name,
      common_test::worlds::kShapesWorld);
    auto checkedOutput = StepWorld<FeaturesStep>(world, true, 1000).first;
    EXPECT_TRUE(checkedOutput);
  }
}

// The features that an engine must have to be loaded by this loader.
struct FeaturesFalling : public  gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::GetModelFromWorld,
  gz::physics::GetLinkFromModel,
  gz::physics::ForwardStep
> {};

template <class T>
class SimulationFeaturesFallingTest :
  public SimulationFeaturesTest<T>{};
using SimulationFeaturesFallingTestTypes =
  ::testing::Types<FeaturesFalling>;
TYPED_TEST_SUITE(SimulationFeaturesFallingTest,
                 SimulationFeaturesFallingTestTypes);

/////////////////////////////////////////////////
TYPED_TEST(SimulationFeaturesFallingTest, Falling)
{
  for (const std::string &name : this->pluginNames)
  {
    CHECK_UNSUPPORTED_ENGINE(name, "tpe")

#ifdef _WIN32
    // See https://github.com/gazebosim/gz-physics/issues/483
    CHECK_UNSUPPORTED_ENGINE(name, "bullet")
#endif
    auto world = LoadPluginAndWorld<FeaturesFalling>(
        this->loader, name,
        common_test::worlds::kFallingWorld);

    auto [checkedOutput, output] =
      StepWorld<FeaturesFalling>(world, true, 1000);
    EXPECT_TRUE(checkedOutput);

    const std::vector<gz::physics::WorldPose> worldPoses =
      output.template Get<gz::physics::WorldPoses>().entries;

    ASSERT_GE(worldPoses.size(), 1);
    const auto link = world->GetModel(0)->GetLink("sphere_link");

    // get the pose of the link from the list of worldPoses
    auto poseIt = std::find_if(worldPoses.begin(), worldPoses.end(),
        [&](const auto &_wPose)
        { return _wPose.body == link->EntityID(); });
    ASSERT_NE(poseIt, worldPoses.end());
    auto pos = poseIt->pose.Pos();
    EXPECT_NEAR(pos.Z(), 1.0, 5e-2);
  }
}


// The features that an engine must have to be loaded by this loader.
struct FeaturesShapeFeatures : gz::physics::FeatureList<
  gz::physics::sdf::ConstructSdfWorld,
  gz::physics::GetModelFromWorld,
  gz::physics::GetLinkFromModel,
  gz::physics::GetShapeFromLink,
  gz::physics::GetModelBoundingBox,
  gz::physics::ForwardStep,

  gz::physics::AttachBoxShapeFeature,
  gz::physics::AttachSphereShapeFeature,
  gz::physics::AttachConeShapeFeature,
  gz::physics::AttachCylinderShapeFeature,
  gz::physics::AttachEllipsoidShapeFeature,
  gz::physics::AttachCapsuleShapeFeature,
  gz::physics::GetSphereShapeProperties,
  gz::physics::GetBoxShapeProperties,
  gz::physics::GetConeShapeProperties,
  gz::physics::GetCylinderShapeProperties,
  gz::physics::GetCapsuleShapeProperties,
  gz::physics::GetEllipsoidShapeProperties
> {};

template <class T>
class SimulationFeaturesShapeFeaturesTest :
  public SimulationFeaturesTest<T>{};
using SimulationFeaturesShapeFeaturesTestTypes =
  ::testing::Types<FeaturesShapeFeatures>;
TYPED_TEST_SUITE(SimulationFeaturesShapeFeaturesTest,
                 SimulationFeaturesShapeFeaturesTestTypes);

/////////////////////////////////////////////////
TYPED_TEST(SimulationFeaturesShapeFeaturesTest, ShapeFeatures)
{
  for (const std::string &name : this->pluginNames)
  {
#ifdef _WIN32
    // See https://github.com/gazebosim/gz-physics/issues/483
    CHECK_UNSUPPORTED_ENGINE(name, "bullet-featherstone")
#endif
    auto world = LoadPluginAndWorld<FeaturesShapeFeatures>(
        this->loader, name,
        common_test::worlds::kShapesWorld);
    std::cerr << "world model count " << world->GetModelCount() << '\n';
    // test ShapeFeatures
    auto sphere = world->GetModel("sphere");
    EXPECT_NE(nullptr, sphere);
    auto sphereLink = sphere->GetLink(0);
    EXPECT_NE(nullptr, sphereLink);
    auto sphereCollision = sphereLink->GetShape(0);
    EXPECT_NE(nullptr, sphereCollision);
    auto sphereShape = sphereCollision->CastToSphereShape();
    EXPECT_NE(nullptr, sphereShape);
    EXPECT_NEAR(1.0, sphereShape->GetRadius(), 1e-6);

    EXPECT_EQ(1u, sphereLink->GetShapeCount());
    auto sphere2 = sphereLink->AttachSphereShape(
        "sphere2", 1.0, Eigen::Isometry3d::Identity());
    EXPECT_EQ(2u, sphereLink->GetShapeCount());
    EXPECT_EQ(sphere2, sphereLink->GetShape(1));

    auto box = world->GetModel("box");
    EXPECT_NE(nullptr, box);
    auto boxLink = box->GetLink("box_link");
    EXPECT_NE(nullptr, boxLink);
    auto boxCollision = boxLink->GetShape(0);
    EXPECT_NE(nullptr, boxCollision);
    auto boxShape = boxCollision->CastToBoxShape();
    EXPECT_NE(nullptr, boxShape);
    EXPECT_EQ(gz::math::Vector3d(100, 100, 1),
        gz::math::eigen3::convert(boxShape->GetSize()));

    auto box2 = boxLink->AttachBoxShape(
      "box2",
      gz::math::eigen3::convert(
        gz::math::Vector3d(1.2, 1.2, 1.2)),
        Eigen::Isometry3d::Identity());
    EXPECT_EQ(2u, boxLink->GetShapeCount());
    EXPECT_EQ(box2, boxLink->GetShape(1));
    EXPECT_EQ(gz::math::Vector3d(1.2, 1.2, 1.2),
              gz::math::eigen3::convert(boxLink->GetShape(1)->CastToBoxShape()->GetSize()));

    auto cylinder = world->GetModel("cylinder");
    auto cylinderLink = cylinder->GetLink(0);
    auto cylinderCollision = cylinderLink->GetShape(0);
    auto cylinderShape = cylinderCollision->CastToCylinderShape();
    ASSERT_NE(nullptr, cylinderShape);
    EXPECT_NEAR(0.5, cylinderShape->GetRadius(), 1e-6);
    EXPECT_NEAR(1.1, cylinderShape->GetHeight(), 1e-6);

    auto cylinder2 = cylinderLink->AttachCylinderShape(
        "cylinder2", 3.0, 4.0, Eigen::Isometry3d::Identity());
    EXPECT_EQ(2u, cylinderLink->GetShapeCount());
    EXPECT_EQ(cylinder2, cylinderLink->GetShape(1));
    ASSERT_NE(nullptr, cylinderLink->GetShape(1)->CastToCylinderShape());
    EXPECT_NEAR(3.0,
        cylinderLink->GetShape(1)->CastToCylinderShape()->GetRadius(),
        1e-6);
    EXPECT_NEAR(4.0,
        cylinderLink->GetShape(1)->CastToCylinderShape()->GetHeight(),
        1e-6);

    auto ellipsoid = world->GetModel("ellipsoid");
    auto ellipsoidLink = ellipsoid->GetLink(0);
    auto ellipsoidCollision = ellipsoidLink->GetShape(0);
    auto ellipsoidShape = ellipsoidCollision->CastToEllipsoidShape();
    ASSERT_NE(nullptr, ellipsoidShape);
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
    ASSERT_NE(nullptr, capsuleShape);
    EXPECT_NEAR(0.2, capsuleShape->GetRadius(), 1e-6);
    EXPECT_NEAR(0.6, capsuleShape->GetLength(), 1e-6);

    auto capsule2 = capsuleLink->AttachCapsuleShape(
        "capsule2", 0.2, 0.6, Eigen::Isometry3d::Identity());
    EXPECT_EQ(2u, capsuleLink->GetShapeCount());
    EXPECT_EQ(capsule2, capsuleLink->GetShape(1));

    auto cone = world->GetModel("cone");
    auto coneLink = cone->GetLink(0);
    auto coneCollision = coneLink->GetShape(0);
    auto coneShape = coneCollision->CastToConeShape();
    ASSERT_NE(nullptr, coneShape);
    EXPECT_NEAR(0.5, coneShape->GetRadius(), 1e-6);
    EXPECT_NEAR(1.1, coneShape->GetHeight(), 1e-6);

    auto cone2 = coneLink->AttachConeShape(
        "cone2", 3.0, 4.0, Eigen::Isometry3d::Identity());
    EXPECT_EQ(2u, coneLink->GetShapeCount());
    EXPECT_EQ(cone2, coneLink->GetShape(1));
    ASSERT_NE(nullptr, coneLink->GetShape(1)->CastToConeShape());
    EXPECT_NEAR(3.0,
        coneLink->GetShape(1)->CastToConeShape()->GetRadius(),
        1e-6);
    EXPECT_NEAR(4.0,
        coneLink->GetShape(1)->CastToConeShape()->GetHeight(),
        1e-6);

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
    auto coneAABB =
      coneCollision->GetAxisAlignedBoundingBox(*coneCollision);

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
    EXPECT_EQ(gz::math::Vector3d(-0.5, -0.5, -0.55),
        gz::math::eigen3::convert(coneAABB).Min());
    EXPECT_EQ(gz::math::Vector3d(0.5, 0.5, 0.55),
        gz::math::eigen3::convert(coneAABB).Max());

    // check model AABB. By default, the AABBs are in world frame
    auto sphereModelAABB = sphere->GetAxisAlignedBoundingBox();
    auto boxModelAABB = box->GetAxisAlignedBoundingBox();
    auto cylinderModelAABB = cylinder->GetAxisAlignedBoundingBox();
    auto ellipsoidModelAABB = ellipsoid->GetAxisAlignedBoundingBox();
    auto capsuleModelAABB = capsule->GetAxisAlignedBoundingBox();
    auto coneModelAABB = cone->GetAxisAlignedBoundingBox();

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
    EXPECT_EQ(gz::math::Vector3d(-3, -9.5, -1.5),
        gz::math::eigen3::convert(coneModelAABB).Min());
    EXPECT_EQ(gz::math::Vector3d(3, -3.5, 2.5),
        gz::math::eigen3::convert(coneModelAABB).Max());
  }
}

struct FreeGroupFeatures : gz::physics::FeatureList<
  gz::physics::FindFreeGroupFeature,
  gz::physics::SetFreeGroupWorldPose,
  gz::physics::SetFreeGroupWorldVelocity,

  gz::physics::GetModelFromWorld,
  gz::physics::GetLinkFromModel,

  gz::physics::FreeGroupFrameSemantics,
  gz::physics::LinkFrameSemantics,

  gz::physics::sdf::ConstructSdfWorld,

  gz::physics::ForwardStep
> {};

template <class T>
class SimulationFeaturesTestFreeGroup :
  public SimulationFeaturesTest<T>{};
using SimulationFeaturesTestFreeGroupTypes =
  ::testing::Types<FreeGroupFeatures>;
TYPED_TEST_SUITE(SimulationFeaturesTestFreeGroup,
                 SimulationFeaturesTestFreeGroupTypes);

TYPED_TEST(SimulationFeaturesTestFreeGroup, FreeGroup)
{
  for (const std::string &name : this->pluginNames)
  {
    auto world = LoadPluginAndWorld<FreeGroupFeatures>(
      this->loader,
      name,
      common_test::worlds::kSphereSdf);

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

    StepWorld<FreeGroupFeatures>(world, true);

    // Set initial pose.
    const gz::math::Pose3d initialPose{0, 0, 2, 0, 0, 0};
    freeGroup->SetWorldPose(
      gz::math::eigen3::convert(initialPose));

    // Set initial velocities.
    const Eigen::Vector3d initialLinearVelocity{0.1, 0.2, 0.3};
    const Eigen::Vector3d initialAngularVelocity{0.4, 0.5, 0.6};
    freeGroup->SetWorldLinearVelocity(initialLinearVelocity);
    freeGroup->SetWorldAngularVelocity(initialAngularVelocity);

    auto freeGroupFrameData = freeGroup->FrameDataRelativeToWorld();
    auto linkFrameData = model->GetLink(0)->FrameDataRelativeToWorld();

    // Before stepping, check that pose matches what was set.
    EXPECT_EQ(initialPose,
              gz::math::eigen3::convert(freeGroupFrameData.pose));
    EXPECT_EQ(initialPose,
              gz::math::eigen3::convert(linkFrameData.pose));

    // Before stepping, check that velocities match what was set.
    AssertVectorApprox vectorPredicateVelocity(1e-7);
    EXPECT_PRED_FORMAT2(vectorPredicateVelocity,
      initialLinearVelocity,
      freeGroupFrameData.linearVelocity);
    EXPECT_PRED_FORMAT2(vectorPredicateVelocity,
      initialLinearVelocity,
      linkFrameData.linearVelocity);
    EXPECT_PRED_FORMAT2(vectorPredicateVelocity,
      initialAngularVelocity,
      freeGroupFrameData.angularVelocity);
    EXPECT_PRED_FORMAT2(vectorPredicateVelocity,
      initialAngularVelocity,
      linkFrameData.angularVelocity);

    // Step the world
    StepWorld<FreeGroupFeatures>(world, false);

    // Check the velocities again.
    freeGroupFrameData = freeGroup->FrameDataRelativeToWorld();
    linkFrameData = model->GetLink(0)->FrameDataRelativeToWorld();
    EXPECT_PRED_FORMAT2(vectorPredicateVelocity,
      initialLinearVelocity,
      freeGroupFrameData.linearVelocity);
    EXPECT_PRED_FORMAT2(vectorPredicateVelocity,
      initialLinearVelocity,
      linkFrameData.linearVelocity);
    EXPECT_PRED_FORMAT2(vectorPredicateVelocity,
      initialAngularVelocity,
      freeGroupFrameData.angularVelocity);
    EXPECT_PRED_FORMAT2(vectorPredicateVelocity,
      initialAngularVelocity,
      linkFrameData.angularVelocity);
  }
}

template <class T>
class SimulationFeaturesTestBasic :
  public SimulationFeaturesTest<T>{};
using SimulationFeaturesTestBasicTypes =
  ::testing::Types<Features>;
TYPED_TEST_SUITE(SimulationFeaturesTestBasic,
                 SimulationFeaturesTestBasicTypes);

TYPED_TEST(SimulationFeaturesTestBasic, ShapeBoundingBox)
{
  for (const std::string &name : this->pluginNames)
  {
    auto world = LoadPluginAndWorld<Features>(
      this->loader,
      name,
      common_test::worlds::kFallingWorld);
    auto sphere = world->GetModel("sphere");
    auto sphereCollision = sphere->GetLink(0)->GetShape(0);
    auto ground = world->GetModel("box");
    auto groundCollision = ground->GetLink(0)->GetShape(0);

    // Test the bounding boxes in the local frames
    auto sphereAABB =
        sphereCollision->GetAxisAlignedBoundingBox(*sphereCollision);

    auto groundAABB =
        groundCollision->GetAxisAlignedBoundingBox(*groundCollision);

    EXPECT_EQ(gz::math::Vector3d(-1, -1, -1),
              gz::math::eigen3::convert(sphereAABB).Min());
    EXPECT_EQ(gz::math::Vector3d(1, 1, 1),
              gz::math::eigen3::convert(sphereAABB).Max());
    EXPECT_EQ(gz::math::Vector3d(-50, -50, -0.5),
              gz::math::eigen3::convert(groundAABB).Min());
    EXPECT_EQ(gz::math::Vector3d(50, 50, 0.5),
              gz::math::eigen3::convert(groundAABB).Max());

    // Test the bounding boxes in the world frames
    sphereAABB = sphereCollision->GetAxisAlignedBoundingBox();
    groundAABB = groundCollision->GetAxisAlignedBoundingBox();

    // The sphere shape has a radius of 1.0, so its bounding box will have
    // dimensions of 1.0 x 1.0 x 1.0. When that bounding box is transformed by
    // a 45-degree rotation, the dimensions that are orthogonal to the axis of
    // rotation will dilate from 1.0 to sqrt(2).
    const double d = std::sqrt(2);
    EXPECT_EQ(gz::math::Vector3d(-d, -1, 2.0 - d),
              gz::math::eigen3::convert(sphereAABB).Min());
    EXPECT_EQ(gz::math::Vector3d(d, 1, 2 + d),
              gz::math::eigen3::convert(sphereAABB).Max());
    EXPECT_EQ(gz::math::Vector3d(-50*d, -50*d, -1),
              gz::math::eigen3::convert(groundAABB).Min());
    EXPECT_EQ(gz::math::Vector3d(50*d, 50*d, 0),
              gz::math::eigen3::convert(groundAABB).Max());
  }
}

TYPED_TEST(SimulationFeaturesTestBasic, CollideBitmasks)
{
  for (const std::string &name : this->pluginNames)
  {
    auto world = LoadPluginAndWorld<Features>(
      this->loader,
      name,
      common_test::worlds::kShapesBitmaskWorld);

    auto baseBox = world->GetModel("box_base");
    auto filteredBox = world->GetModel("box_filtered");
    auto collidingBox = world->GetModel("box_colliding");

    auto checkedOutput = StepWorld<Features>(world, true).first;
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
    checkedOutput = StepWorld<Features>(world, false).first;
    EXPECT_FALSE(checkedOutput);
    contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(0u, contacts.size());

    // Now remove both filter masks (no collisions will be filtered)
    // Equivalent to 0xFF
    collidingShape->RemoveCollisionFilterMask();
    filteredShape->RemoveCollisionFilterMask();
    checkedOutput = StepWorld<Features>(world, false).first;
    EXPECT_FALSE(checkedOutput);
    // Expect box_filtered and box_colliding to collide with box_base
    contacts = world->GetContactsFromLastStep();
    EXPECT_NE(0u, contacts.size());
  }
}


TYPED_TEST(SimulationFeaturesTestBasic, RetrieveContacts)
{
  for (const std::string &name : this->pluginNames)
  {
  auto world =
    LoadPluginAndWorld<Features>(
      this->loader,
      name,
      common_test::worlds::kShapesWorld);

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

    auto cone = world->GetModel("cone");
    auto coneFreeGroup = cone->FindFreeGroup();
    EXPECT_NE(nullptr, coneFreeGroup);

    auto box = world->GetModel("box");

    // step and get contacts
    auto checkedOutput = StepWorld<Features>(world, true).first;
    EXPECT_TRUE(checkedOutput);
    auto contacts = world->GetContactsFromLastStep();

    // large box in the middle should be intersecting with sphere, cylinder,
    // capsule and ellipsoid
    EXPECT_NE(0u, contacts.size());
    unsigned int contactBoxSphere = 0u;
    unsigned int contactBoxCylinder = 0u;
    unsigned int contactBoxCapsule = 0u;
    unsigned int contactBoxEllipsoid = 0u;
    unsigned int contactBoxCone = 0u;

    for (auto &contact : contacts)
    {
      const auto &contactPoint =
          contact.template Get<gz::physics::World3d<Features>::ContactPoint>();
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
      else if ((m1->GetName() == "box" && m2->GetName() == "cone") ||
          (m1->GetName() == "cone" && m2->GetName() == "box"))
      {
        contactBoxCone++;
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
    EXPECT_NE(0u, contactBoxCone);

    // move sphere away
    sphereFreeGroup->SetWorldPose(gz::math::eigen3::convert(
        gz::math::Pose3d(0, 100, 0.5, 0, 0, 0)));

    // step and get contacts
    checkedOutput = StepWorld<Features>(world, false).first;
    EXPECT_FALSE(checkedOutput);
    contacts = world->GetContactsFromLastStep();

    // large box in the middle should be intersecting with cylinder, capsule,
    // ellipsoid, cone
    EXPECT_NE(0u, contacts.size());

    contactBoxCylinder = 0u;
    contactBoxCapsule = 0u;
    contactBoxEllipsoid = 0u;
    contactBoxCone = 0u;
    for (auto contact : contacts)
    {
      const auto &contactPoint =
          contact.template Get<gz::physics::World3d<Features>::ContactPoint>();
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
      else if ((m1->GetName() == "box" && m2->GetName() == "cone") ||
          (m1->GetName() == "cone" && m2->GetName() == "box"))
      {
        contactBoxCone++;
      }
      else
      {
        FAIL() << "There should only be contacts between box and cylinder";
      }
    }
    EXPECT_NE(0u, contactBoxCylinder);
    EXPECT_NE(0u, contactBoxCapsule);
    EXPECT_NE(0u, contactBoxEllipsoid);
    EXPECT_NE(0u, contactBoxCone);

    // move cylinder away
    cylinderFreeGroup->SetWorldPose(gz::math::eigen3::convert(
        gz::math::Pose3d(0, -100, 0.5, 0, 0, 0)));

    // move capsule away
    capsuleFreeGroup->SetWorldPose(gz::math::eigen3::convert(
        gz::math::Pose3d(0, -100, 100, 0, 0, 0)));

    // move ellipsoid away
    ellipsoidFreeGroup->SetWorldPose(gz::math::eigen3::convert(
        gz::math::Pose3d(0, -100, -100, 0, 0, 0)));

    // move cone away
    coneFreeGroup->SetWorldPose(gz::math::eigen3::convert(
        gz::math::Pose3d(100, -100, 0.5, 0, 0, 0)));

    // step and get contacts
    checkedOutput = StepWorld<Features>(world, false).first;
    EXPECT_FALSE(checkedOutput);
    contacts = world->GetContactsFromLastStep();

    // no entities should be colliding
    EXPECT_TRUE(contacts.empty());
  }
}

struct FeaturesContactPropertiesCallback : gz::physics::FeatureList<
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

  #ifdef DART_HAS_CONTACT_SURFACE
      gz::physics::SetContactPropertiesCallbackFeature,
  #endif

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
> {};

#ifdef DART_HAS_CONTACT_SURFACE
using ContactSurfaceParams =
  gz::physics::SetContactPropertiesCallbackFeature::
    ContactSurfaceParams<gz::physics::World3d<FeaturesContactPropertiesCallback>::Policy>;
#endif

template <class T>
class SimulationFeaturesTestFeaturesContactPropertiesCallback :
  public SimulationFeaturesTest<T>{};
using SimulationFeaturesTestFeaturesContactPropertiesCallbackTypes =
  ::testing::Types<FeaturesContactPropertiesCallback>;
TYPED_TEST_SUITE(SimulationFeaturesTestFeaturesContactPropertiesCallback,
                 FeaturesContactPropertiesCallback);

/////////////////////////////////////////////////
TYPED_TEST(SimulationFeaturesTestFeaturesContactPropertiesCallback, ContactPropertiesCallback)
{
  for (const std::string &name : this->pluginNames)
  {
    auto world =
      LoadPluginAndWorld<FeaturesContactPropertiesCallback>(
          this->loader,
          name,
          common_test::worlds::kContactSdf);

    auto sphere = world->GetModel("sphere");
    auto groundPlane = world->GetModel("ground_plane");
    auto groundPlaneCollision = groundPlane->GetLink(0)->GetShape(0);

    // Use a set because the order of collisions is not determined.
    std::set<gz::physics::Shape3dPtr<FeaturesContactPropertiesCallback>> possibleCollisions = {
      groundPlaneCollision,
      sphere->GetLink(0)->GetShape(0),
      sphere->GetLink(1)->GetShape(0),
      sphere->GetLink(2)->GetShape(0),
      sphere->GetLink(3)->GetShape(0),
    };
    std::map<gz::physics::Shape3dPtr<FeaturesContactPropertiesCallback>, Eigen::Vector3d> expectations
    {
      {sphere->GetLink(0)->GetShape(0), {0.0, 0.0, 0.0}},
        {sphere->GetLink(1)->GetShape(0), {0.0, 1.0, 0.0}},
        {sphere->GetLink(2)->GetShape(0), {1.0, 0.0, 0.0}},
        {sphere->GetLink(3)->GetShape(0), {1.0, 1.0, 0.0}},
    };

    const double gravity = 9.8;
    std::map<gz::physics::Shape3dPtr<FeaturesContactPropertiesCallback>, double> forceExpectations
    {
      // Contact force expectations are: link mass * gravity.
      {sphere->GetLink(0)->GetShape(0), 0.1 * gravity},
        {sphere->GetLink(1)->GetShape(0), 1.0 * gravity},
        {sphere->GetLink(2)->GetShape(0), 2.0 * gravity},
        {sphere->GetLink(3)->GetShape(0), 3.0 * gravity},
    };

    // This procedure checks the validity of a generated contact point. It is
    // used both when checking the contacts after the step is finished and for
    // checking them inside the contact joint properties callback. The callback
    // is called after the contacts are generated but before they affect the
    // physics. That is why contact force is zero during the callback.
    auto checkContact = [&](
        const gz::physics::World3d<FeaturesContactPropertiesCallback>::Contact& _contact,
        const bool zeroForce)
    {
      const auto &contactPoint =
        _contact.Get<gz::physics::World3d<FeaturesContactPropertiesCallback>::ContactPoint>();
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

      EXPECT_TRUE(gz::physics::test::Equal(expectedContactPos,
            contactPoint.point, 1e-6));

      // Check if the engine populated the extra contact data struct
      const auto* extraContactData =
        _contact.Query<gz::physics::World3d<FeaturesContactPropertiesCallback>::ExtraContactData>();
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
          zeroForce ? 0 : forceExpectations.at(testCollision), 1e-3);
    };

#ifdef DART_HAS_CONTACT_SURFACE
    size_t numContactCallbackCalls = 0u;
    auto contactCallback = [&](
        const gz::physics::World3d<FeaturesContactPropertiesCallback>::Contact& _contact,
        size_t _numContactsOnCollision,
        ContactSurfaceParams& _surfaceParams)
    {
      numContactCallbackCalls++;
      checkContact(_contact, true);
      EXPECT_EQ(1u, _numContactsOnCollision);
      // the values in _surfaceParams are implemented as std::optional to allow
      // physics engines fill only those parameters that are actually
      // implemented
      ASSERT_TRUE(_surfaceParams.frictionCoeff.has_value());
      ASSERT_TRUE(_surfaceParams.secondaryFrictionCoeff.has_value());
      // not implemented in DART yet
      EXPECT_FALSE(_surfaceParams.rollingFrictionCoeff.has_value());
      // not implemented in DART yet
      EXPECT_FALSE(_surfaceParams.secondaryRollingFrictionCoeff.has_value());
      // not implemented in DART yet
      EXPECT_FALSE(_surfaceParams.torsionalFrictionCoeff.has_value());
      ASSERT_TRUE(_surfaceParams.slipCompliance.has_value());
      ASSERT_TRUE(_surfaceParams.secondarySlipCompliance.has_value());
      ASSERT_TRUE(_surfaceParams.restitutionCoeff.has_value());
      ASSERT_TRUE(_surfaceParams.firstFrictionalDirection.has_value());
      ASSERT_TRUE(_surfaceParams.contactSurfaceMotionVelocity.has_value());
      // these constraint parameters are implemented in DART but are not filled
      // when the callback is called; they are only read after the callback ends
      EXPECT_FALSE(_surfaceParams.errorReductionParameter.has_value());
      EXPECT_FALSE(_surfaceParams.maxErrorReductionVelocity.has_value());
      EXPECT_FALSE(_surfaceParams.maxErrorAllowance.has_value());
      EXPECT_FALSE(_surfaceParams.constraintForceMixing.has_value());

      EXPECT_NEAR(_surfaceParams.frictionCoeff.value(), 1.0, 1e-6);
      EXPECT_NEAR(_surfaceParams.secondaryFrictionCoeff.value(), 1.0, 1e-6);
      EXPECT_NEAR(_surfaceParams.slipCompliance.value(), 0.0, 1e-6);
      EXPECT_NEAR(_surfaceParams.secondarySlipCompliance.value(), 0.0, 1e-6);
      EXPECT_NEAR(_surfaceParams.restitutionCoeff.value(), 0.0, 1e-6);

      EXPECT_TRUE(gz::physics::test::Equal(Eigen::Vector3d(0, 0, 1),
            _surfaceParams.firstFrictionalDirection.value(), 1e-6));

      EXPECT_TRUE(gz::physics::test::Equal(Eigen::Vector3d(0, 0, 0),
            _surfaceParams.contactSurfaceMotionVelocity.value(), 1e-6));
    };
    world->AddContactPropertiesCallback("test", contactCallback);
#endif

    // The first step already has contacts, but the contact force due to the
    // impact does not match the steady-state force generated by the
    // body's weight.
    StepWorld<FeaturesContactPropertiesCallback>(world, true);

#ifdef DART_HAS_CONTACT_SURFACE
    // There are 4 collision bodies in the world all colliding at the same time
    EXPECT_EQ(4u, numContactCallbackCalls);
#endif

    // After a second step, the contact force reaches steady-state
    StepWorld<FeaturesContactPropertiesCallback>(world, false);

#ifdef DART_HAS_CONTACT_SURFACE
    // There are 4 collision bodies in the world all colliding at the same time
    EXPECT_EQ(8u, numContactCallbackCalls);
#endif

    auto contacts = world->GetContactsFromLastStep();
    if(this->PhysicsEngineName(name) != "tpe")
    {
      EXPECT_EQ(4u, contacts.size());
    }

    for (auto &contact : contacts)
    {
      checkContact(contact, false);
    }

#ifdef DART_HAS_CONTACT_SURFACE
    // removing a non-existing callback yields no error but returns false
    EXPECT_FALSE(world->RemoveContactPropertiesCallback("foo"));

    // removing an existing callback works and the callback is no longer called
    EXPECT_TRUE(world->RemoveContactPropertiesCallback("test"));

    // Third step
    StepWorld<FeaturesContactPropertiesCallback>(world, false);

    // Number of callback calls is the same as after the 2nd call
    EXPECT_EQ(8u, numContactCallbackCalls);

    // Now we check that changing _surfaceParams inside the contact properties
    // callback affects the result of the simulation; we set
    // contactSurfaceMotionVelocity to [1,0,0] which accelerates the contact
    // points from 0 m/s to 1 m/s in a single simulation step.

    auto contactCallback2 = [&](
        const gz::physics::World3d<FeaturesContactPropertiesCallback>::Contact& /*_contact*/,
        size_t /*_numContactsOnCollision*/,
        ContactSurfaceParams& _surfaceParams)
    {
      numContactCallbackCalls++;
      // friction direction is [0,0,1] and contact surface motion velocity uses
      // the X value to denote the desired velocity along the friction direction
      _surfaceParams.contactSurfaceMotionVelocity->x() = 1.0;
    };
    world->AddContactPropertiesCallback("test2", contactCallback2);

    numContactCallbackCalls = 0u;
    // Fourth step
    StepWorld<FeaturesContactPropertiesCallback>(world, false);
    EXPECT_EQ(4u, numContactCallbackCalls);

    // Adjust the expected forces to account for the added acceleration along Z
    forceExpectations =
    {
      // Contact force expectations are:
      // link mass * (gravity + acceleration to 1 m.s^-1 in 1 ms)
      {sphere->GetLink(0)->GetShape(0), 0.1 * gravity + 100},
      {sphere->GetLink(1)->GetShape(0), 1.0 * gravity + 999.99},
      {sphere->GetLink(2)->GetShape(0), 2.0 * gravity + 1999.98},
      {sphere->GetLink(3)->GetShape(0), 3.0 * gravity + 2999.97},
    };

    // Verify that the detected contacts correspond to the adjusted expectations
    contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(4u, contacts.size());
    for (auto &contact : contacts)
    {
      checkContact(contact, false);
    }

    EXPECT_TRUE(world->RemoveContactPropertiesCallback("test2"));
#endif
  }
}

TYPED_TEST(SimulationFeaturesTestBasic, MultipleCollisions)
{
  for (const std::string &name : this->pluginNames)
  {
    CHECK_UNSUPPORTED_ENGINE(name, "tpe")

    auto world = LoadPluginAndWorld<Features>(
      this->loader,
      name,
      common_test::worlds::kMultipleCollisionsSdf);

    // model free group test
    auto model = world->GetModel("box");
    auto freeGroup = model->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroup);
    GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
      ASSERT_NE(nullptr, freeGroup->CanonicalLink());
    GZ_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION
      ASSERT_NE(nullptr, freeGroup->RootLink());

    auto link = model->GetLink("box_link");
    auto freeGroupLink = link->FindFreeGroup();
    ASSERT_NE(nullptr, freeGroupLink);

    StepWorld<Features>(world, true);

    auto frameData = model->GetLink(0)->FrameDataRelativeToWorld();
    EXPECT_EQ(gz::math::Pose3d(0, 0, 4, 0, 0, 0),
        gz::math::eigen3::convert(frameData.pose));

    StepWorld<Features>(world, false, 1000);
    frameData = model->GetLink(0)->FrameDataRelativeToWorld();
    gz::math::Pose3d framePose = gz::math::eigen3::convert(frameData.pose);

    EXPECT_NEAR(0.5, framePose.Z(), 0.1);
  }
}

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);

  if (!SimulationFeaturesTest<Features>::init(
       argc, argv))
    return -1;
  return RUN_ALL_TESTS();
}
