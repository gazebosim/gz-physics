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

#include <cstddef>
#include <string>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <gz/plugin/Loader.hh>
#include <gz/physics/FeatureList.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

struct TestFeatures : public gz::physics::FeatureList<
    gz::physics::ForwardStep,
    gz::physics::GetBasicJointState,
    gz::physics::GetJointFromModel,
    gz::physics::GetModelFromWorld,
    gz::physics::SetBasicJointState,
    gz::physics::SetJointVelocityCommandFeature,
    gz::physics::sdf::ConstructSdfWorld>
{
};

// A ball joint followed by a revolute joint. Gravity is off so only the
// commands move the model.
const std::string kSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <gravity>0 0 0</gravity>
    <model name="m">
      <joint name="fix_to_world" type="fixed">
        <parent>world</parent>
        <child>base</child>
      </joint>
      <link name="base"/>
      <link name="link1">
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz></inertia>
        </inertial>
      </link>
      <link name="link2">
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz></inertia>
        </inertial>
      </link>
      <joint name="ball" type="ball">
        <parent>base</parent>
        <child>link1</child>
      </joint>
      <joint name="hinge" type="revolute">
        <parent>link1</parent>
        <child>link2</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>
    </model>
  </world>
</sdf>)";

auto LoadWorld()
{
  gz::plugin::Loader loader;
  loader.LoadLib(mujoco_plugin_LIB);
  auto plugin = loader.Instantiate("gz::physics::mujoco::Plugin");
  auto engine = gz::physics::RequestEngine3d<TestFeatures>::From(plugin);

  sdf::Root root;
  EXPECT_TRUE(root.LoadSdfString(kSdf).empty());
  return engine->ConstructWorld(*root.WorldByIndex(0));
}

// Regression test for gz-physics#1085: each ball joint DOF must have its own
// actuator, so a command on DOF n reaches ball axis n and not a later joint.
TEST(BallJointActuators, ForceReachesOnlyTheCommandedBallDof)
{
  auto world = LoadWorld();
  ASSERT_TRUE(world);
  auto model = world->GetModel("m");
  ASSERT_TRUE(model);
  auto ball = model->GetJoint("ball");
  auto hinge = model->GetJoint("hinge");
  ASSERT_TRUE(ball);
  ASSERT_TRUE(hinge);

  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;

  for (std::size_t dof = 0; dof < 3; ++dof)
  {
    ball->SetForce(dof, 10.0);
    world->Step(output, state, input);

    // After a step, GetForce reports the generalized actuator force each DOF
    // received during that step.
    for (std::size_t i = 0; i < 3; ++i)
    {
      EXPECT_DOUBLE_EQ(i == dof ? 10.0 : 0.0, ball->GetForce(i))
          << "commanded ball DOF " << dof << ", checked ball DOF " << i;
    }
    EXPECT_DOUBLE_EQ(0.0, hinge->GetForce(0))
        << "commanding ball DOF " << dof << " drove the revolute joint";
  }
}

TEST(BallJointActuators, VelocityCommandReachesEachBallDof)
{
  auto world = LoadWorld();
  ASSERT_TRUE(world);
  auto model = world->GetModel("m");
  ASSERT_TRUE(model);
  auto ball = model->GetJoint("ball");
  auto hinge = model->GetJoint("hinge");
  ASSERT_TRUE(ball);
  ASSERT_TRUE(hinge);

  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;

  for (std::size_t dof = 0; dof < 3; ++dof)
  {
    for (int step = 0; step < 10; ++step)
    {
      ball->SetVelocityCommand(dof, 1.0);
      world->Step(output, state, input);
    }

    EXPECT_NEAR(1.0, ball->GetVelocity(dof), 1e-2)
        << "velocity command did not reach ball DOF " << dof;
    // The commands drive the ball's actuators only, never the revolute one.
    EXPECT_DOUBLE_EQ(0.0, hinge->GetForce(0))
        << "commanding ball DOF " << dof << " drove the revolute joint";
  }
}
