/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>
#include <gz/plugin/Loader.hh>
#include <sdf/Root.hh>

namespace
{
struct Features : gz::physics::FeatureList<gz::physics::GetModelFromWorld,
  gz::physics::GetJointFromModel, gz::physics::GetBasicJointProperties,
  gz::physics::GetBasicJointState, gz::physics::SetBasicJointState,
  gz::physics::SetJointEffortLimitsFeature,
  gz::physics::SetJointVelocityCommandFeature, gz::physics::ForwardStep,
  gz::physics::sdf::ConstructSdfWorld> {};

auto MakeWorld()
{
  gz::plugin::Loader loader;
  loader.LoadLib(bullet_featherstone_plugin_LIB);
  auto engine = gz::physics::RequestEngine3d<Features>::From(
      loader.Instantiate("gz::physics::bullet_featherstone::Plugin"));
  sdf::Root root;
  root.LoadSdfString(R"(<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><model name='m'>
    <link name='base'/><joint name='fixed' type='fixed'><parent>world</parent><child>base</child></joint>
    <link name='child'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial></link>
    <joint name='ball' type='ball'><parent>base</parent><child>child</child></joint>
  </model></world></sdf>)");
  return std::make_pair(engine, engine->ConstructWorld(*root.WorldByIndex(0)));
}
}

TEST(BulletFeatherstoneBallMotor, CommandsRequestedAxis)
{
  auto [engine, world] = MakeWorld();
  auto ball = world->GetModel("m")->GetJoint("ball");
  ball->SetMaxEffort(1, 100.0);

  gz::physics::ForwardStep::Output out;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Input in;

  const std::size_t numSteps = 15;
  for (std::size_t i = 0; i < numSteps; ++i)
  {
    ball->SetVelocityCommand(1, 1.0);
    world->Step(out, state, in);
  }

  EXPECT_NEAR(1.0, ball->GetVelocity(1), 1e-2);
  EXPECT_NEAR(0.0, ball->GetVelocity(0), 1e-2);
  EXPECT_NEAR(0.0, ball->GetVelocity(2), 1e-2);
}

TEST(BulletFeatherstoneBallMotor, SimultaneousMultiAxisCommands)
{
  auto [engine, world] = MakeWorld();
  auto ball = world->GetModel("m")->GetJoint("ball");
  ball->SetMaxEffort(0, 100.0);
  ball->SetMaxEffort(1, 100.0);

  gz::physics::ForwardStep::Output out;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Input in;

  const std::size_t numSteps = 15;
  for (std::size_t i = 0; i < numSteps; ++i)
  {
    ball->SetVelocityCommand(0, 1.0);
    ball->SetVelocityCommand(1, -0.5);
    world->Step(out, state, in);
  }

  EXPECT_NEAR(1.0, ball->GetVelocity(0), 1e-2);
  EXPECT_NEAR(-0.5, ball->GetVelocity(1), 1e-2);
  EXPECT_NEAR(0.0, ball->GetVelocity(2), 1e-2);
}

TEST(BulletFeatherstoneBallMotor, PerDofEffortLimits)
{
  auto [engine, world] = MakeWorld();
  auto ball = world->GetModel("m")->GetJoint("ball");
  ball->SetMaxEffort(0, 100.0);
  ball->SetMaxEffort(1, 50.0);

  gz::physics::ForwardStep::Output out;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Input in;

  ball->SetVelocityCommand(0, 1.0);
  ball->SetVelocityCommand(1, 1.0);
  world->Step(out, state, in);

  // In 1 step (dt = 0.001 s, I = 1.0):
  // Axis 0: max impulse = 100 * 0.001 = 0.1 -> delta_v = 0.1
  // Axis 1: max impulse = 50 * 0.001 = 0.05 -> delta_v = 0.05
  EXPECT_NEAR(0.1, ball->GetVelocity(0), 1e-3);
  EXPECT_NEAR(0.05, ball->GetVelocity(1), 1e-3);
}

TEST(BulletFeatherstoneBallMotor, TransitionToForceControl)
{
  auto [engine, world] = MakeWorld();
  auto ball = world->GetModel("m")->GetJoint("ball");
  ball->SetMaxEffort(1, 100.0);

  gz::physics::ForwardStep::Output out;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Input in;

  for (std::size_t i = 0; i < 10; ++i)
  {
    ball->SetVelocityCommand(1, 1.0);
    world->Step(out, state, in);
  }
  EXPECT_NEAR(1.0, ball->GetVelocity(1), 1e-2);

  // Setting force should disable joint velocity control
  ball->SetForce(1, 0.0);
  for (std::size_t i = 0; i < 5; ++i)
  {
    world->Step(out, state, in);
  }
  // With 0 torque and no damping/gravity, angular velocity should coast
  // near 1.0
  EXPECT_NEAR(1.0, ball->GetVelocity(1), 1e-2);
}

TEST(BulletFeatherstoneBallMotor, DirectForceControl)
{
  auto [engine, world] = MakeWorld();
  auto ball = world->GetModel("m")->GetJoint("ball");
  ball->SetMaxEffort(1, 100.0);
  ball->SetForce(1, 100.0);
  gz::physics::ForwardStep::Output out;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Input in;
  world->Step(out, state, in);
  std::cout << "force axis0=" << ball->GetVelocity(0)
            << " axis1=" << ball->GetVelocity(1) << std::endl;
  EXPECT_GT(std::abs(ball->GetVelocity(1)), 1e-3);
  EXPECT_NEAR(0.0, ball->GetVelocity(0), 1e-3);
}

TEST(BulletFeatherstoneBallMotor, IndependentVelocityAndForceControl)
{
  auto [engine, world] = MakeWorld();
  auto ball = world->GetModel("m")->GetJoint("ball");
  ball->SetMaxEffort(0, 100.0);
  ball->SetMaxEffort(1, 100.0);

  gz::physics::ForwardStep::Output out;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Input in;

  // Set velocity command on axis 0 and direct force on axis 1
  for (std::size_t i = 0; i < 15; ++i)
  {
    ball->SetVelocityCommand(0, 1.0);
    ball->SetForce(1, 10.0);
    world->Step(out, state, in);
  }

  // Axis 0 should reach its velocity target (1.0)
  EXPECT_NEAR(1.0, ball->GetVelocity(0), 1e-2);
  // Axis 1 should have been accelerated by continuous force
  EXPECT_GT(ball->GetVelocity(1), 0.1);
  // Axis 2 remains unactuated
  EXPECT_NEAR(0.0, ball->GetVelocity(2), 1e-2);
}
