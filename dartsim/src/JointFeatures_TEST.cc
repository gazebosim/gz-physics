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
#include <ignition/physics/FixedJoint.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/physics/RevoluteJoint.hh>
#include <ignition/physics/dartsim/World.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <limits>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "test/Utils.hh"

using namespace ignition;

using TestFeatureList = ignition::physics::FeatureList<
  physics::dartsim::RetrieveWorld,
  physics::AttachFixedJointFeature,
  physics::DetachJointFeature,
  physics::SetJointTransformFromParentFeature,
  physics::ForwardStep,
  physics::FreeJointCast,
  physics::GetBasicJointState,
  physics::GetEntities,
  physics::RevoluteJointCast,
  physics::SetBasicJointState,
  physics::SetJointVelocityCommandFeature,
  physics::sdf::ConstructSdfModel,
  physics::sdf::ConstructSdfWorld,
  physics::SetJointPositionLimitsFeature,
  physics::SetJointVelocityLimitsFeature,
  physics::SetJointEffortLimitsFeature
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

  const auto *dartBaseLink = skeleton->getBodyNode("base");
  ASSERT_NE(nullptr, dartBaseLink);
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

  // Check that invalid velocity commands don't cause collisions to fail
  for (std::size_t i = 0; i < 1000; ++i)
  {
    joint->SetForce(0, std::numeric_limits<double>::quiet_NaN());
    // expect the position of the pendulum to stay aabove ground
    world->Step(output, state, input);
    EXPECT_NEAR(0.0, dartBaseLink->getWorldTransform().translation().z(), 1e-3);
  }

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

  // Check that invalid velocity commands don't cause collisions to fail
  for (std::size_t i = 0; i < 1000; ++i)
  {
    joint->SetVelocityCommand(0, std::numeric_limits<double>::quiet_NaN());
    // expect the position of the pendulum to stay aabove ground
    world->Step(output, state, input);
    EXPECT_NEAR(0.0, dartBaseLink->getWorldTransform().translation().z(), 1e-3);
  }
}

TEST_F(JointFeaturesFixture, JointSetPositionLimitsWithForceControl)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(TEST_WORLD_DIR "test.world");
  ASSERT_TRUE(errors.empty()) << errors.front();

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));
  auto model = world->GetModel("simple_joint_test");
  auto joint = model->GetJoint("j1");

  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  world->Step(output, state, input);

  auto pos = joint->GetPosition(0);

  joint->SetMinPosition(0, pos - 0.1);
  joint->SetMaxPosition(0, pos + 0.1);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetForce(0, 100);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(pos + 0.1, joint->GetPosition(0), 1e-3);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetForce(0, -100);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(pos - 0.1, joint->GetPosition(0), 1e-3);

  joint->SetMinPosition(0, pos - 0.5);
  joint->SetMaxPosition(0, pos + 0.5);

  for (std::size_t i = 0; i < 300; ++i)
  {
    joint->SetForce(0, 100);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(pos + 0.5, joint->GetPosition(0), 1e-2);

  for (std::size_t i = 0; i < 300; ++i)
  {
    joint->SetForce(0, -100);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(pos - 0.5, joint->GetPosition(0), 1e-2);

  joint->SetMinPosition(0, -math::INF_D);
  joint->SetMaxPosition(0, math::INF_D);
  joint->SetPosition(0, pos);

  for (std::size_t i = 0; i < 300; ++i)
  {
    joint->SetForce(0, 100);
    world->Step(output, state, input);
  }
  EXPECT_LT(pos + 0.5, joint->GetPosition(0));
}

TEST_F(JointFeaturesFixture, JointSetVelocityLimitsWithForceControl)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(TEST_WORLD_DIR "test.world");
  ASSERT_TRUE(errors.empty()) << errors.front();

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));
  auto model = world->GetModel("simple_joint_test");
  auto joint = model->GetJoint("j1");

  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  world->Step(output, state, input);

  joint->SetMinVelocity(0, -0.25);
  joint->SetMaxVelocity(0, 0.5);

  for (std::size_t i = 0; i < 10; ++i)
  {
    joint->SetForce(0, 1000);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(0.5, joint->GetVelocity(0), 1e-6);

  for (std::size_t i = 0; i < 10; ++i)
  {
    joint->SetForce(0, -1000);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(-0.25, joint->GetVelocity(0), 1e-6);

  // set minimum velocity above zero
  joint->SetMinVelocity(0, 0.25);

  for (std::size_t i = 0; i < 10; ++i)
  {
    joint->SetForce(0, 0);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(0.25, joint->GetVelocity(0), 1e-6);

  for (std::size_t i = 0; i < 10; ++i)
  {
    // make sure the minimum velocity is kept even without velocity commands
    world->Step(output, state, input);
  }
  EXPECT_NEAR(0.25, joint->GetVelocity(0), 1e-6);

  joint->SetMinVelocity(0, -0.25);
  joint->SetPosition(0, 0);
  joint->SetVelocity(0, 0);

  for (std::size_t i = 0; i < 10; ++i)
  {
    joint->SetForce(0, 0);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

  joint->SetMinVelocity(0, -math::INF_D);
  joint->SetMaxVelocity(0, math::INF_D);

  for (std::size_t i = 0; i < 10; ++i)
  {
    joint->SetForce(0, 1000);
    world->Step(output, state, input);
  }
  EXPECT_LT(0.5, joint->GetVelocity(0));
}

TEST_F(JointFeaturesFixture, JointSetEffortLimitsWithForceControl)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(TEST_WORLD_DIR "test.world");
  ASSERT_TRUE(errors.empty()) << errors.front();

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));
  auto model = world->GetModel("simple_joint_test");
  auto joint = model->GetJoint("j1");

  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  world->Step(output, state, input);

  auto pos = joint->GetPosition(0);

  joint->SetMinEffort(0, -1e-6);
  joint->SetMaxEffort(0, 1e-6);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetForce(0, 1);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(pos, joint->GetPosition(0), 1e-3);
  EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetForce(0, -1);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(pos, joint->GetPosition(0), 1e-3);
  EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

  joint->SetMinEffort(0, -80);
  joint->SetMaxEffort(0, 80);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetForce(0, 1);
    world->Step(output, state, input);
  }
  EXPECT_LT(pos, joint->GetPosition(0));
  EXPECT_LT(0, joint->GetVelocity(0));

  joint->SetMinEffort(0, -math::INF_D);
  joint->SetMaxEffort(0, math::INF_D);
  joint->SetPosition(0, 0);
  joint->SetVelocity(0, 0);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetForce(0, 1);
    world->Step(output, state, input);
  }
  EXPECT_LT(pos, joint->GetPosition(0));
  EXPECT_LT(0, joint->GetVelocity(0));
}

TEST_F(JointFeaturesFixture, JointSetCombinedLimitsWithForceControl)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(TEST_WORLD_DIR "test.world");
  ASSERT_TRUE(errors.empty()) << errors.front();

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));
  auto model = world->GetModel("simple_joint_test");
  auto joint = model->GetJoint("j1");

  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  world->Step(output, state, input);

  auto pos = joint->GetPosition(0);

  joint->SetMinPosition(0, pos - 0.1);
  joint->SetMaxPosition(0, pos + 0.1);
  joint->SetMinVelocity(0, -0.25);
  joint->SetMaxVelocity(0, 0.5);
  joint->SetMinEffort(0, -1e-6);
  joint->SetMaxEffort(0, 1e-6);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetForce(0, 100);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(pos, joint->GetPosition(0), 1e-2);
  EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetForce(0, -100);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(pos, joint->GetPosition(0), 1e-2);
  EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

  joint->SetMinEffort(0, -500);
  joint->SetMaxEffort(0, 1000);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetForce(0, 1000);
    world->Step(output, state, input);
  }
  // 0.05 because we go 0.1 s with max speed 0.5
  EXPECT_NEAR(pos + 0.05, joint->GetPosition(0), 1e-2);
  EXPECT_NEAR(0.5, joint->GetVelocity(0), 1e-6);

  for (std::size_t i = 0; i < 200; ++i)
  {
    joint->SetForce(0, 1000);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(pos + 0.1, joint->GetPosition(0), 1e-2);
  EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

  joint->SetPosition(0, pos);
  EXPECT_NEAR(pos, joint->GetPosition(0), 1e-2);

  joint->SetMinVelocity(0, -1);
  joint->SetMaxVelocity(0, 1);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetForce(0, 1000);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(pos + 0.1, joint->GetPosition(0), 1e-2);
  EXPECT_NEAR(1, joint->GetVelocity(0), 1e-6);

  joint->SetPosition(0, pos);
  EXPECT_NEAR(pos, joint->GetPosition(0), 1e-2);

  joint->SetMinPosition(0, -1e6);
  joint->SetMaxPosition(0, 1e6);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetForce(0, 1000);
    world->Step(output, state, input);
  }
  EXPECT_LT(pos + 0.1, joint->GetPosition(0));
  EXPECT_NEAR(1, joint->GetVelocity(0), 1e-6);
}

// TODO(anyone): position limits do not work very well with velocity control
// bug https://github.com/dartsim/dart/issues/1583
// resolved in DART 6.11.0
TEST_F(JointFeaturesFixture, DISABLED_JointSetPositionLimitsWithVelocityControl)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(TEST_WORLD_DIR "test.world");
  ASSERT_TRUE(errors.empty()) << errors.front();

  const std::string modelName{"simple_joint_test"};
  const std::string jointName{"j1"};

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));

  auto model = world->GetModel(modelName);
  auto joint = model->GetJoint(jointName);

  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  world->Step(output, state, input);

  auto pos = joint->GetPosition(0);

  joint->SetMinPosition(0, pos - 0.1);
  joint->SetMaxPosition(0, pos + 0.1);
  for (std::size_t i = 0; i < 1000; ++i)
  {
    joint->SetVelocityCommand(0, 1);
    world->Step(output, state, input);

    if (i % 500 == 499)
    {
      EXPECT_NEAR(pos + 0.1, joint->GetPosition(0), 1e-2);
      EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);
    }
  }
}

TEST_F(JointFeaturesFixture, JointSetVelocityLimitsWithVelocityControl)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(TEST_WORLD_DIR "test.world");
  ASSERT_TRUE(errors.empty()) << errors.front();

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));
  auto model = world->GetModel("simple_joint_test");
  auto joint = model->GetJoint("j1");

  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  joint->SetMinVelocity(0, -0.1);
  joint->SetMaxVelocity(0, 0.1);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetVelocityCommand(0, 1);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(0.1, joint->GetVelocity(0), 1e-6);

  for (std::size_t i = 0; i < 10; ++i)
  {
    joint->SetVelocityCommand(0, 0.1);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(0.1, joint->GetVelocity(0), 1e-6);

  for (std::size_t i = 0; i < 10; ++i)
  {
    joint->SetVelocityCommand(0, -0.025);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(-0.025, joint->GetVelocity(0), 1e-6);

  for (std::size_t i = 0; i < 10; ++i)
  {
    joint->SetVelocityCommand(0, -1);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(-0.1, joint->GetVelocity(0), 1e-6);

  joint->SetMinVelocity(0, -math::INF_D);
  joint->SetMaxVelocity(0, math::INF_D);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetVelocityCommand(0, 1);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(1, joint->GetVelocity(0), 1e-6);
}

TEST_F(JointFeaturesFixture, JointSetEffortLimitsWithVelocityControl)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(TEST_WORLD_DIR "test.world");
  ASSERT_TRUE(errors.empty()) << errors.front();

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));
  auto model = world->GetModel("simple_joint_test");
  auto joint = model->GetJoint("j1");

  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  joint->SetMinEffort(0, -1e-6);
  joint->SetMaxEffort(0, 1e-6);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetVelocityCommand(0, 1);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

  joint->SetMinEffort(0, -80);
  joint->SetMaxEffort(0, 80);

  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetVelocityCommand(0, -1);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(-1, joint->GetVelocity(0), 1e-6);

  joint->SetMinEffort(0, -math::INF_D);
  joint->SetMaxEffort(0, math::INF_D);

  for (std::size_t i = 0; i < 10; ++i)
  {
    joint->SetVelocityCommand(0, -100);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(-100, joint->GetVelocity(0), 1e-6);
}

TEST_F(JointFeaturesFixture, JointSetCombinedLimitsWithVelocityControl)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(TEST_WORLD_DIR "test.world");
  ASSERT_TRUE(errors.empty()) << errors.front();

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));
  auto model = world->GetModel("simple_joint_test");
  auto joint = model->GetJoint("j1");

  // Test joint velocity command
  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  joint->SetMinVelocity(0, -0.5);
  joint->SetMaxVelocity(0, 0.5);
  joint->SetMinEffort(0, -1e-6);
  joint->SetMaxEffort(0, 1e-6);

  for (std::size_t i = 0; i < 1000; ++i)
  {
    joint->SetVelocityCommand(0, 1);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(0, joint->GetVelocity(0), 1e-6);

  joint->SetMinEffort(0, -1e6);
  joint->SetMaxEffort(0, 1e6);

  for (std::size_t i = 0; i < 1000; ++i)
  {
    joint->SetVelocityCommand(0, -1);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(-0.5, joint->GetVelocity(0), 1e-6);
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

  // Detach() can be called again though it has no effect
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
// Attach a fixed joint between links that belong to different models
TEST_F(JointFeaturesFixture, JointAttachDetach)
{
  sdf::Root root;
  const sdf::Errors errors =
      root.Load(TEST_WORLD_DIR "joint_across_models.sdf");
  ASSERT_TRUE(errors.empty()) << errors.front();

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));
  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const std::string modelName1{"M1"};
  const std::string modelName2{"M2"};
  const std::string bodyName{"body"};

  auto model1 = world->GetModel(modelName1);
  auto model2 = world->GetModel(modelName2);
  auto model1Body = model1->GetLink(bodyName);
  auto model2Body = model2->GetLink(bodyName);

  const dart::dynamics::SkeletonPtr skeleton1 =
      dartWorld->getSkeleton(modelName1);
  const dart::dynamics::SkeletonPtr skeleton2 =
      dartWorld->getSkeleton(modelName2);
  ASSERT_NE(nullptr, skeleton1);
  ASSERT_NE(nullptr, skeleton2);

  auto *dartBody1 = skeleton1->getBodyNode(bodyName);
  auto *dartBody2 = skeleton2->getBodyNode(bodyName);

  ASSERT_NE(nullptr, dartBody1);
  ASSERT_NE(nullptr, dartBody2);

  const math::Pose3d initialModel1Pose(0, 0, 0.25, 0, 0, 0);
  const math::Pose3d initialModel2Pose(0, 0, 3.0, 0, 0, 0);

  EXPECT_EQ(initialModel1Pose,
            math::eigen3::convert(dartBody1->getWorldTransform()));
  EXPECT_EQ(initialModel2Pose,
            math::eigen3::convert(dartBody2->getWorldTransform()));

  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  const std::size_t numSteps = 100;
  for (std::size_t i = 0; i < numSteps; ++i)
  {
    world->Step(output, state, input);

    // Expect the model1 to stay at rest (since it's on the ground) and model2
    // to start falling
    math::Vector3d body1LinearVelocity =
        math::eigen3::convert(dartBody1->getLinearVelocity());
    math::Vector3d body2LinearVelocity =
        math::eigen3::convert(dartBody2->getLinearVelocity());
    EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-7);
    // Negative z velocity
    EXPECT_GT(0.0, body2LinearVelocity.Z());
  }

  const auto poseParent = dartBody1->getTransform();
  const auto poseChild = dartBody2->getTransform();
  auto poseParentChild = poseParent.inverse() * poseChild;

  auto fixedJoint = model2Body->AttachFixedJoint(model1Body);

  // AttachFixedJoint snaps the child body to the origin of the parent, so we
  // set a transform on the joint to keep the transform between the two bodies
  // the same as it was before they were attached
  fixedJoint->SetTransformFromParent(poseParentChild);

  // The name of the link obtained using the ign-physics API should remain the
  // same even though AttachFixedJoint renames the associated BodyNode.
  EXPECT_EQ(bodyName, model2Body->GetName());

  for (std::size_t i = 0; i < numSteps; ++i)
  {
    world->Step(output, state, input);

    // Expect the model1 to remain at rest and model2
    // to stop moving
    math::Vector3d body1LinearVelocity =
        math::eigen3::convert(dartBody1->getLinearVelocity());
    math::Vector3d body2LinearVelocity =
        math::eigen3::convert(dartBody2->getLinearVelocity());
    EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-7);
    EXPECT_NEAR(0.0, body2LinearVelocity.Z(), 1e-7);
  }

  // now detach joint and expect model2 to start moving again
  fixedJoint->Detach();

  // The name of the link obtained using the ign-physics API should remain the
  // same even though Detach renames the associated BodyNode.
  EXPECT_EQ(bodyName, model2Body->GetName());

  for (std::size_t i = 0; i < numSteps; ++i)
  {
    world->Step(output, state, input);

    // Expect the model1 to remain at rest and model2
    // to start moving again
    math::Vector3d body1LinearVelocity =
        math::eigen3::convert(dartBody1->getLinearVelocity());
    math::Vector3d body2LinearVelocity =
        math::eigen3::convert(dartBody2->getLinearVelocity());
    EXPECT_NEAR(0.0, body1LinearVelocity.Z(), 1e-7);
    // Negative z velocity
    EXPECT_GT(0.0, body2LinearVelocity.Z());
  }

  // After a while, body2 should reach the ground and come to a stop
  for (std::size_t i = 0; i < 1000; ++i)
  {
    world->Step(output, state, input);
  }

  EXPECT_NEAR(0.0, dartBody2->getLinearVelocity().z(), 1e-3);
}

/////////////////////////////////////////////////
// Expectations on number of links before/after attach/detach
TEST_F(JointFeaturesFixture, LinkCountsInJointAttachDetach)
{
  sdf::Root root;
  const sdf::Errors errors =
      root.Load(TEST_WORLD_DIR "joint_across_models.sdf");
  ASSERT_TRUE(errors.empty()) << errors.front();

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));
  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const std::string modelName1{"M1"};
  const std::string modelName2{"M2"};
  const std::string bodyName{"body"};

  auto model1 = world->GetModel(modelName1);
  auto model2 = world->GetModel(modelName2);
  auto model1Body = model1->GetLink(bodyName);
  auto model2Body = model2->GetLink(bodyName);

  // Before attaching we expect each model to have 1 link
  EXPECT_EQ(1u, model1->GetLinkCount());
  EXPECT_EQ(1u, model2->GetLinkCount());

  auto fixedJoint = model2Body->AttachFixedJoint(model1Body);

  // After attaching we expect each model to have 1 link, but the current
  // behavior is that there are 2 links in model1 and 0 in model2
  // EXPECT_EQ(1u, model1->GetLinkCount());
  // EXPECT_EQ(1u, model2->GetLinkCount());
  EXPECT_EQ(2u, model1->GetLinkCount());
  EXPECT_EQ(0u, model2->GetLinkCount());

  // now detach joint and expect model2 to start moving again
  fixedJoint->Detach();
  // After detaching we expect each model to have 1 link
  EXPECT_EQ(1u, model1->GetLinkCount());
  EXPECT_EQ(1u, model2->GetLinkCount());

  // Test that a model with the same name as a link doesn't cause problems
  const std::string modelName3{"body"};
  auto model3 = world->GetModel(modelName3);
  EXPECT_EQ(1u, model3->GetLinkCount());

  auto model3Body = model3->GetLink(bodyName);
  auto fixedJoint2 = model3Body->AttachFixedJoint(model2Body);
  EXPECT_EQ(2u, model2->GetLinkCount());
  fixedJoint2->Detach();
  // After detaching we expect each model to have 1 link
  EXPECT_EQ(1u, model2->GetLinkCount());
  EXPECT_EQ(1u, model3->GetLinkCount());
}

/////////////////////////////////////////////////
// Attach a fixed joint between links that belong to different models where one
// of the models is created after a step is called
TEST_F(JointFeaturesFixture, JointAttachDetachSpawnedModel)
{
  std::string model1Str = R"(
  <sdf version="1.6">
    <model name="M1">
      <pose>0 0 0.1 0 0 0</pose>
      <link name="body">
        <collision name="coll_box">
          <geometry>
            <box>
              <size>0.2 0.2 0.2</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>
  </sdf>)";

  std::string model2Str = R"(
  <sdf version="1.6">
    <model name="M2">
      <pose>1 0 0.1 0 0 0</pose>
      <link name="chassis">
        <collision name="coll_sphere">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
        </collision>
      </link>
    </model>
  </sdf>)";

  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  physics::World3dPtr<TestFeatureList> world;
  {
    sdf::Root root;
    const sdf::Errors errors = root.Load(TEST_WORLD_DIR "ground.sdf");
    ASSERT_TRUE(errors.empty()) << errors.front();
    world = this->engine->ConstructWorld(*root.WorldByIndex(0));
    ASSERT_NE(nullptr, world);
  }

  {
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(model1Str);
    ASSERT_TRUE(errors.empty()) << errors.front();
    ASSERT_NE(nullptr, root.ModelByIndex(0));
    world->ConstructModel(*root.ModelByIndex(0));
  }

  world->Step(output, state, input);

  {
    sdf::Root root;
    sdf::Errors errors = root.LoadSdfString(model2Str);
    ASSERT_TRUE(errors.empty()) << errors.front();
    ASSERT_NE(nullptr, root.ModelByIndex(0));
    world->ConstructModel(*root.ModelByIndex(0));
  }

  const std::string modelName1{"M1"};
  const std::string modelName2{"M2"};
  const std::string bodyName1{"body"};
  const std::string bodyName2{"chassis"};

  auto model1 = world->GetModel(modelName1);
  auto model2 = world->GetModel(modelName2);
  auto model1Body = model1->GetLink(bodyName1);
  auto model2Body = model2->GetLink(bodyName2);

  dart::simulation::WorldPtr dartWorld = world->GetDartsimWorld();
  ASSERT_NE(nullptr, dartWorld);

  const auto skeleton1 = dartWorld->getSkeleton(modelName1);
  const auto skeleton2 = dartWorld->getSkeleton(modelName2);
  ASSERT_NE(nullptr, skeleton1);
  ASSERT_NE(nullptr, skeleton2);

  auto *dartBody1 = skeleton1->getBodyNode(bodyName1);
  auto *dartBody2 = skeleton2->getBodyNode(bodyName2);

  ASSERT_NE(nullptr, dartBody1);
  ASSERT_NE(nullptr, dartBody2);

  const auto poseParent = dartBody1->getTransform();
  const auto poseChild = dartBody2->getTransform();

  // Before ign-physics PR #31, uncommenting the following `step` call makes
  // this test pass, but commenting it out makes it fail.
  // world->Step(output, state, input);
  auto fixedJoint = model2Body->AttachFixedJoint(model1Body);

  // Pose of child relative to parent
  auto poseParentChild = poseParent.inverse() * poseChild;

  // We let the joint be at the origin of the child link.
  fixedJoint->SetTransformFromParent(poseParentChild);

  const std::size_t numSteps = 100;

  for (std::size_t i = 0; i < numSteps; ++i)
  {
    world->Step(output, state, input);
  }

  // Expect both bodies to hit the ground and stop
  EXPECT_NEAR(0.0, dartBody1->getLinearVelocity().z(), 1e-3);
  EXPECT_NEAR(0.0, dartBody2->getLinearVelocity().z(), 1e-3);

  fixedJoint->Detach();

  for (std::size_t i = 0; i < numSteps; ++i)
  {
    world->Step(output, state, input);
  }

  // Expect both bodies to remain in contact with the ground with zero velocity.
  EXPECT_NEAR(0.0, dartBody1->getLinearVelocity().z(), 1e-3);
  EXPECT_NEAR(0.0, dartBody2->getLinearVelocity().z(), 1e-3);
}

/////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
