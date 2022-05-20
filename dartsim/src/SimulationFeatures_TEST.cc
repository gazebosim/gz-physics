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

#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>

#include <gz/physics/FindFeatures.hh>
#include <gz/plugin/Loader.hh>
#include <gz/physics/RequestEngine.hh>

// Features
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/GetContacts.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Shape.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/physics/ContactProperties.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <test/PhysicsPluginsList.hh>
#include <test/Utils.hh>

struct TestFeatureList : gz::physics::FeatureList<
    gz::physics::LinkFrameSemantics,
    gz::physics::ForwardStep,
    gz::physics::GetContactsFromLastStepFeature,
    gz::physics::GetEntities,
    gz::physics::GetShapeBoundingBox,
    gz::physics::CollisionFilterMaskFeature,
#ifdef DART_HAS_CONTACT_SURFACE
    gz::physics::SetContactPropertiesCallbackFeature,
#endif
    gz::physics::sdf::ConstructSdfWorld
> { };

using TestWorldPtr = gz::physics::World3dPtr<TestFeatureList>;
using TestShapePtr = gz::physics::Shape3dPtr<TestFeatureList>;
using TestWorld = gz::physics::World3d<TestFeatureList>;
using TestContactPoint = TestWorld::ContactPoint;
using TestExtraContactData = TestWorld::ExtraContactData;
using TestContact = TestWorld::Contact;
#ifdef DART_HAS_CONTACT_SURFACE
using ContactSurfaceParams =
  gz::physics::SetContactPropertiesCallbackFeature::
    ContactSurfaceParams<TestWorld::Policy>;
#endif

std::unordered_set<TestWorldPtr> LoadWorlds(
    const std::string &_library,
    const std::string &_world)
{
  gz::plugin::Loader loader;
  loader.LoadLib(_library);

  const std::set<std::string> pluginNames =
      gz::physics::FindFeatures3d<TestFeatureList>::From(loader);

  EXPECT_LT(0u, pluginNames.size());

  std::unordered_set<TestWorldPtr> worlds;
  for (const std::string &name : pluginNames)
  {
    gz::plugin::PluginPtr plugin = loader.Instantiate(name);

    std::cout << " -- Plugin name: " << name << std::endl;

    auto engine =
        gz::physics::RequestEngine3d<TestFeatureList>::From(plugin);
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
    auto checkedOutput = StepWorld(world, true, 1000);
    EXPECT_TRUE(checkedOutput);

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

    auto checkedOutput = StepWorld(world, true);
    EXPECT_TRUE(checkedOutput);
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
    checkedOutput = StepWorld(world, false);
    EXPECT_FALSE(checkedOutput);
    contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(0u, contacts.size());

    // Now remove both filter masks (no collision will be filtered)
    // Equivalent to set to 0xFF
    collidingShape->RemoveCollisionFilterMask();
    filteredShape->RemoveCollisionFilterMask();
    checkedOutput = StepWorld(world, false);
    EXPECT_FALSE(checkedOutput);
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

    // This procedure checks the validity of a generated contact point. It is
    // used both when checking the contacts after the step is finished and for
    // checking them inside the contact joint properties callback. The callback
    // is called after the contacts are generated but before they affect the
    // physics. That is why contact force is zero during the callback.
    auto checkContact = [&](const TestContact& _contact, const bool zeroForce)
    {
      const auto &contactPoint = _contact.Get<TestContactPoint>();
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
      const auto* extraContactData = _contact.Query<TestExtraContactData>();
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
      const TestContact& _contact,
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
    StepWorld(world, true);

#ifdef DART_HAS_CONTACT_SURFACE
    // There are 4 collision bodies in the world all colliding at the same time
    EXPECT_EQ(4u, numContactCallbackCalls);
#endif

    // After a second step, the contact force reaches steady-state
    StepWorld(world, false);

#ifdef DART_HAS_CONTACT_SURFACE
    // There are 4 collision bodies in the world all colliding at the same time
    EXPECT_EQ(8u, numContactCallbackCalls);
#endif

    auto contacts = world->GetContactsFromLastStep();
    EXPECT_EQ(4u, contacts.size());

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
    StepWorld(world, false);

    // Number of callback calls is the same as after the 2nd call
    EXPECT_EQ(8u, numContactCallbackCalls);

    // Now we check that changing _surfaceParams inside the contact properties
    // callback affects the result of the simulation; we set
    // contactSurfaceMotionVelocity to [1,0,0] which accelerates the contact
    // points from 0 m/s to 1 m/s in a single simulation step.

    auto contactCallback2 = [&](
      const TestContact& /*_contact*/,
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
    StepWorld(world, false);
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

INSTANTIATE_TEST_CASE_P(PhysicsPlugins, SimulationFeatures_TEST,
    ::testing::ValuesIn(gz::physics::test::g_PhysicsPluginLibraries),); // NOLINT

int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
