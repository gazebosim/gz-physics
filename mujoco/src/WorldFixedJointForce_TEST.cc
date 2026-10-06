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

struct TestFeatures : public gz::physics::FeatureList<
    gz::physics::ForwardStep,
    gz::physics::GetBasicJointState,
    gz::physics::GetJointFromModel,
    gz::physics::SetBasicJointState,
    gz::physics::sdf::ConstructSdfWorld,
    gz::physics::sdf::ConstructSdfModel>
{
};

// Regression test: a model added to a running world leaves the MuJoCo spec
// uncompiled until the first step, so a joint command issued before that step
// used to index stale buffers and crash. Build a model incrementally, command a
// force before any step, and check that it does not crash and actually drives
// the joint (a bare bounds guard alone would drop the command instead).
TEST(WorldFixedJointForce, ForceOnIncrementalModelIsAppliedAndDoesNotCrash)
{
  gz::plugin::Loader loader;
  loader.LoadLib(mujoco_plugin_LIB);
  auto plugin = loader.Instantiate("gz::physics::mujoco::Plugin");
  ASSERT_TRUE(plugin);
  auto engine = gz::physics::RequestEngine3d<TestFeatures>::From(plugin);
  ASSERT_TRUE(engine);

  // A revolute joint (joint1) is the one the controller drives; its base link
  // is welded to the world, as on a typical fixed-base manipulator. Gravity is
  // off so the only thing that can move joint1 is the commanded force.
  const std::string sdfString = R"(
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">
    <model name="arm">
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
  const sdf::Model *sdfModel = sdfWorld->ModelByIndex(0);
  ASSERT_NE(nullptr, sdfModel);

  // Build an empty world, then add the model to it, as gz-sim does. Leaving
  // gravity at zero isolates the applied torque.
  sdf::World emptyWorld;
  emptyWorld.SetName(sdfWorld->Name());
  emptyWorld.SetGravity(gz::math::Vector3d::Zero);
  auto world = engine->ConstructWorld(emptyWorld);
  ASSERT_TRUE(world);

  auto model = world->ConstructModel(*sdfModel);
  ASSERT_TRUE(model);

  auto joint = model->GetJoint("joint1");
  ASSERT_TRUE(joint);

  // Before the fix this force, issued while the freshly added model has not
  // been recompiled, wrote out of bounds and crashed the process.
  joint->SetForce(0, 1.0);

  const double startPos = joint->GetPosition(0);

  // Drive the joint like a controller would and let it run.
  gz::physics::ForwardStep::Input input;
  gz::physics::ForwardStep::State state;
  gz::physics::ForwardStep::Output output;
  for (std::size_t i = 0; i < 100; ++i)
  {
    joint->SetForce(0, 1.0);
    world->Step(output, state, input);
  }

  // The force reached the joint's real degree of freedom rather than being
  // dropped, so the joint turned under the constant torque.
  const double endPos = joint->GetPosition(0);
  EXPECT_GT(endPos - startPos, 1e-3)
      << "the commanded force was not applied to the joint";
}
