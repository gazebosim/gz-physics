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
  physics::ForwardStep,
  physics::GetBasicJointState,
  physics::GetEntities,
  physics::SetJointCommandFeature,
  physics::sdf::ConstructSdfWorld
>;

using TestEnginePtr = physics::Engine3dPtr<TestFeatureList>;

class JointFeatureFixture : public ::testing::Test
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
TEST_F(JointFeatureFixture, JointSetCommand)
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

  joint->SetVelocityCommand(0, 1);
  world->Step(output, state, input);
  // Setting a velocity command changes the actuator type to SERVO
  EXPECT_EQ(dart::dynamics::Joint::SERVO, dartJoint->getActuatorType());

  joint->SetAccelerationCommand(0, 1);
  world->Step(output, state, input);
  // Setting an acceleration command changes the actuator type to ACCELERATION
  EXPECT_EQ(dart::dynamics::Joint::ACCELERATION, dartJoint->getActuatorType());

  joint->SetForceCommand(0, 1);
  world->Step(output, state, input);
  // Setting an acceleration command changes the actuator type to FORCE
  EXPECT_EQ(dart::dynamics::Joint::FORCE, dartJoint->getActuatorType());
}

// Test setting joint velocity commands and verify that the commanded velocity
// is acheived
TEST_F(JointFeatureFixture, JointSetVelocityCommand)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(TEST_WORLD_DIR "test.world");
  ASSERT_TRUE(errors.empty()) << errors.front();

  const std::string modelName{"double_pendulum_with_base"};
  const std::string jointName{"upper_joint"};

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));

  auto model = world->GetModel(modelName);
  auto joint = model->GetJoint(jointName);

  // Test joint velocity command
  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  std::size_t numSteps = 10;
  for (std::size_t i = 0; i < numSteps; ++i)
  {
    joint->SetVelocityCommand(0, 1);
    world->Step(output, state, input);
  }
  EXPECT_NEAR(1.0, joint->GetVelocity(0), 1e-6);
}

/////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
