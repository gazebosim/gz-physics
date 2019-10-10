/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 3.0 (the "License");
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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/simulation/World.hpp>

#include <gtest/gtest.h>

#include <iostream>

#include <ignition/physics/FindFeatures.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/physics/RequestEngine.hh>

#include <ignition/math/eigen3/Conversions.hh>

// Features
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/FreeJoint.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/physics/RevoluteJoint.hh>
#include <ignition/physics/dartsim/World.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "test/Utils.hh"

using namespace ignition;

using TestFeatureList = ignition::physics::FeatureList<
  physics::dartsim::RetrieveWorld,
  physics::DetachJointFeature,
  physics::ForwardStep,
  physics::FreeJointCast,
  physics::GetBasicJointState,
  physics::GetEntities,
  physics::RevoluteJointCast,
  physics::SetJointVelocityCommandFeature,
  physics::sdf::ConstructSdfWorld
>;

using TestEnginePtr = physics::Engine3dPtr<TestFeatureList>;

class JointFeaturesFixture : public ::testing::Test
{
  protected: void SetUp() override
  {
    ignition::plugin::Loader loader;
    loader.LoadLib(dartsim_plugin_LIB);

    ignition::plugin::PluginPtr dartsim =
        loader.Instantiate("ignition::physics::dartsim::Plugin");

    this->engine =
        ignition::physics::RequestEngine3d<TestFeatureList>::From(dartsim);
    ASSERT_NE(nullptr, this->engine);
  }
  protected: TestEnginePtr engine;
};

// Test setting joint commands and verify that the joint type is set accordingly
// and that the commanded velocity is acheived
TEST_F(JointFeaturesFixture, JointSetCommand)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(TEST_WORLD_DIR "test.world");
  ASSERT_TRUE(errors.empty()) << errors.front();

  const std::string modelName{"double_pendulum_with_base"};
  const std::string jointName{"upper_joint"};

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));

  auto model = world->GetModel(modelName);
  auto joint = model->GetJoint(jointName);

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const dart::dynamics::SkeletonPtr skeleton =
      dartWorld->getSkeleton(modelName);
  ASSERT_NE(nullptr, skeleton);

  const auto *dartJoint = skeleton->getJoint(jointName);

  // Default actuatore type
  EXPECT_EQ(dart::dynamics::Joint::FORCE, dartJoint->getActuatorType());

  // Test joint velocity command
  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  // Expect negative joint velocity after 1 step without joint command
  world->Step(output, state, input);
  EXPECT_LT(joint->GetVelocity(0), 0.0);

  joint->SetVelocityCommand(0, 1);
  world->Step(output, state, input);
  // Setting a velocity command changes the actuator type to SERVO
  EXPECT_EQ(dart::dynamics::Joint::SERVO, dartJoint->getActuatorType());

  const std::size_t numSteps = 10;
  for (std::size_t i = 0; i < numSteps; ++i)
  {
    // Call SetVelocityCommand before each step
    joint->SetVelocityCommand(0, 1);
    world->Step(output, state, input);
    EXPECT_NEAR(1.0, joint->GetVelocity(0), 1e-6);
  }

  for (std::size_t i = 0; i < numSteps; ++i)
  {
    // expect joint to freeze in subsequent steps without SetVelocityCommand
    world->Step(output, state, input);
    EXPECT_NEAR(0.0, joint->GetVelocity(0), 1e-6);
  }
}

// Test detaching joints.
TEST_F(JointFeaturesFixture, JointDetach)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(TEST_WORLD_DIR "test.world");
  ASSERT_TRUE(errors.empty()) << errors.front();

  const std::string modelName{"double_pendulum_with_base"};
  const std::string upperJointName{"upper_joint"};
  const std::string lowerJointName{"lower_joint"};
  const std::string upperLinkName{"upper_link"};
  const std::string lowerLinkName{"lower_link"};

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));

  auto model = world->GetModel(modelName);
  auto upperLink = model->GetLink(upperLinkName);
  auto lowerLink = model->GetLink(lowerLinkName);
  auto upperJoint = model->GetJoint(upperJointName);
  auto lowerJoint = model->GetJoint(lowerJointName);

  // test Cast*Joint functions
  EXPECT_NE(nullptr, upperJoint->CastToRevoluteJoint());
  EXPECT_NE(nullptr, lowerJoint->CastToRevoluteJoint());
  EXPECT_EQ(nullptr, upperJoint->CastToFreeJoint());
  EXPECT_EQ(nullptr, lowerJoint->CastToFreeJoint());

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const dart::dynamics::SkeletonPtr skeleton =
      dartWorld->getSkeleton(modelName);
  ASSERT_NE(nullptr, skeleton);

  const auto *dartUpperLink = skeleton->getBodyNode(upperLinkName);
  const auto *dartLowerLink = skeleton->getBodyNode(lowerLinkName);
  EXPECT_EQ("RevoluteJoint", dartUpperLink->getParentJoint()->getType());
  EXPECT_EQ("RevoluteJoint", dartLowerLink->getParentJoint()->getType());

  const math::Pose3d initialUpperLinkPose(1, 0, 2.1, -IGN_PI/2, 0, 0);
  const math::Pose3d initialLowerLinkPose(1.25, 1, 2.1, -2, 0, 0);

  EXPECT_EQ(initialUpperLinkPose,
            math::eigen3::convert(dartUpperLink->getWorldTransform()));
  EXPECT_EQ(initialLowerLinkPose,
            math::eigen3::convert(dartLowerLink->getWorldTransform()));

  // detach lower joint
  lowerJoint->Detach();
  EXPECT_EQ("FreeJoint", dartLowerLink->getParentJoint()->getType());
  EXPECT_NE(nullptr, lowerJoint->CastToFreeJoint());
  EXPECT_EQ(nullptr, lowerJoint->CastToRevoluteJoint());

  // expect poses to remain unchanged
  EXPECT_EQ(initialUpperLinkPose,
            math::eigen3::convert(dartUpperLink->getWorldTransform()));
  EXPECT_EQ(initialLowerLinkPose,
            math::eigen3::convert(dartLowerLink->getWorldTransform()));

  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  const std::size_t numSteps = 10;
  for (std::size_t i = 0; i < numSteps; ++i)
  {
    // step forward and expect lower link to fall
    world->Step(output, state, input);

    // expect upper link to rotate
    EXPECT_LT(upperJoint->GetVelocity(0), 0.0);

    // expect lower link to fall down without rotating
    math::Vector3d lowerLinkLinearVelocity =
        math::eigen3::convert(dartLowerLink->getLinearVelocity());
    EXPECT_NEAR(0.0, lowerLinkLinearVelocity.X(), 1e-10);
    EXPECT_NEAR(0.0, lowerLinkLinearVelocity.Y(), 1e-10);
    EXPECT_GT(0.0, lowerLinkLinearVelocity.Z());
    math::Vector3d lowerLinkAngularVelocity =
        math::eigen3::convert(dartLowerLink->getAngularVelocity());
    EXPECT_EQ(math::Vector3d::Zero, lowerLinkAngularVelocity);
  }

  // now detach the upper joint too, and ensure that velocity is preserved
  math::Pose3d upperLinkPose =
      math::eigen3::convert(dartUpperLink->getWorldTransform());
  math::Vector3d upperLinkLinearVelocity =
      math::eigen3::convert(dartUpperLink->getLinearVelocity());
  math::Vector3d upperLinkAngularVelocity =
      math::eigen3::convert(dartUpperLink->getAngularVelocity());
  // sanity check on velocity values
  EXPECT_LT(1e-5, upperLinkLinearVelocity.Z());
  EXPECT_GT(-0.03, upperLinkAngularVelocity.X());
  EXPECT_NEAR(0.0, upperLinkLinearVelocity.X(), 1e-6);
  EXPECT_NEAR(0.0, upperLinkLinearVelocity.Y(), 1e-6);
  EXPECT_NEAR(0.0, upperLinkAngularVelocity.Y(), 1e-6);
  EXPECT_NEAR(0.0, upperLinkAngularVelocity.Z(), 1e-6);

  upperJoint->Detach();
  EXPECT_EQ("FreeJoint", dartUpperLink->getParentJoint()->getType());
  EXPECT_NE(nullptr, upperJoint->CastToFreeJoint());
  EXPECT_EQ(nullptr, upperJoint->CastToRevoluteJoint());

  EXPECT_EQ(upperLinkPose,
      math::eigen3::convert(dartUpperLink->getWorldTransform()));
  EXPECT_EQ(upperLinkLinearVelocity,
      math::eigen3::convert(dartUpperLink->getLinearVelocity()));
  EXPECT_EQ(upperLinkAngularVelocity,
      math::eigen3::convert(dartUpperLink->getAngularVelocity()));
}

/////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
