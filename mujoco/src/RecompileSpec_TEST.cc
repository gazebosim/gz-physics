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

#include <mujoco/mujoco.h>

#include <string>

#include <gz/math/Vector3.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <gz/plugin/Loader.hh>
#include <gz/physics/FeatureList.hh>
#include <gz/physics/ForwardStep.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Joint.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include "Base.hh"

struct TestFeatures : public gz::physics::FeatureList<
    gz::physics::ForwardStep,
    gz::physics::GetBasicJointState,
    gz::physics::GetJointFromModel,
    gz::physics::SetBasicJointState,
    gz::physics::sdf::ConstructSdfWorld,
    gz::physics::sdf::ConstructSdfModel>
{
};

// Regression test: with MuJoCo before 3.14, mj_recompile read the controls of
// the actuators added since the last compilation out of bounds, so they
// started with whatever memory followed the old ctrl array ("Nan, Inf or huge
// value in CTRL" in the first step). Add a joint to a world that already
// compiled one, with the memory after the old ctrl array filled with a huge
// value, and check that the new actuator starts at zero while the control of
// the existing one is kept.
TEST(RecompileSpec, NewActuatorsStartWithZeroControl)
{
  gz::plugin::Loader loader;
  loader.LoadLib(mujoco_plugin_LIB);
  auto plugin = loader.Instantiate("gz::physics::mujoco::Plugin");
  ASSERT_TRUE(plugin);
  auto engine = gz::physics::RequestEngine3d<TestFeatures>::From(plugin);
  ASSERT_TRUE(engine);

  // Two arms, each with one revolute joint (and so one actuator) driven by
  // its own link, welded to the world.
  const std::string sdfString = R"(
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <model name="arm1">
      <joint name="fix_to_world" type="fixed">
        <parent>world</parent>
        <child>link0</child>
      </joint>
      <link name="link0"/>
      <link name="link1">
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz></inertia>
        </inertial>
      </link>
      <joint name="joint1" type="revolute">
        <parent>link0</parent>
        <child>link1</child>
        <axis><xyz>1 0 0</xyz></axis>
      </joint>
    </model>
    <model name="arm2">
      <pose>1 0 0 0 0 0</pose>
      <joint name="fix_to_world" type="fixed">
        <parent>world</parent>
        <child>link0</child>
      </joint>
      <link name="link0"/>
      <link name="link1">
        <inertial>
          <mass>1.0</mass>
          <inertia><ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz></inertia>
        </inertial>
      </link>
      <joint name="joint1" type="revolute">
        <parent>link0</parent>
        <child>link1</child>
        <axis><xyz>1 0 0</xyz></axis>
      </joint>
    </model>
  </world>
</sdf>)";

  sdf::Root root;
  ASSERT_TRUE(root.LoadSdfString(sdfString).empty());
  const sdf::World *sdfWorld = root.WorldByIndex(0);
  ASSERT_NE(nullptr, sdfWorld);
  const sdf::Model *sdfArm1 = sdfWorld->ModelByName("arm1");
  const sdf::Model *sdfArm2 = sdfWorld->ModelByName("arm2");
  ASSERT_NE(nullptr, sdfArm1);
  ASSERT_NE(nullptr, sdfArm2);

  // Build an empty world, then add the models to it one by one, as gz-sim
  // does.
  sdf::World emptyWorld;
  emptyWorld.SetName(sdfWorld->Name());
  emptyWorld.SetGravity(gz::math::Vector3d::Zero);
  auto world = engine->ConstructWorld(emptyWorld);
  ASSERT_TRUE(world);

  auto *worldInfo = static_cast<gz::physics::mujoco::WorldInfo *>(
      world->FullIdentity().ref.get());

  auto arm1 = world->ConstructModel(*sdfArm1);
  ASSERT_TRUE(arm1);
  auto joint1 = arm1->GetJoint("joint1");
  ASSERT_TRUE(joint1);

  // Setting a force recompiles the world, which has one actuator now.
  joint1->SetForce(0, 3.0);
  ASSERT_NE(nullptr, worldInfo->mjModelObj);
  ASSERT_NE(nullptr, worldInfo->mjDataObj);
  ASSERT_EQ(1, worldInfo->mjModelObj->nu);
  ASSERT_DOUBLE_EQ(3.0, worldInfo->mjDataObj->ctrl[0]);

  // Fill the memory from the end of the ctrl array to the end of the array
  // that follows it with a huge value, which is what the recompilation reads
  // for an actuator that did not exist.
  mjData *oldData = worldInfo->mjDataObj;
  for (mjtNum *value = oldData->ctrl + worldInfo->mjModelObj->nu;
       value < oldData->qfrc_applied + worldInfo->mjModelObj->nv; ++value)
  {
    *value = 1e30;
  }

  auto arm2 = world->ConstructModel(*sdfArm2);
  ASSERT_TRUE(arm2);
  auto joint2 = arm2->GetJoint("joint1");
  ASSERT_TRUE(joint2);

  // Reading a position recompiles the world, which has two actuators now.
  joint2->GetPosition(0);

  const mjModel *m = worldInfo->mjModelObj;
  const mjData *d = worldInfo->mjDataObj;
  ASSERT_EQ(2, m->nu);

  // The new actuator starts at zero, and the control of the existing one is
  // kept. A huge control would also make the forward dynamics at the end of
  // the recompilation warn about bad controls.
  EXPECT_EQ(0.0, d->ctrl[1]);
  EXPECT_DOUBLE_EQ(3.0, d->ctrl[0]);
  EXPECT_EQ(0, d->warning[mjWARN_BADCTRL].number);

  // The first step runs without bad controls.
  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;
  world->Step(output, state, input);
  EXPECT_EQ(0, worldInfo->mjDataObj->warning[mjWARN_BADCTRL].number);
}
