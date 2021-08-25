/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include <iostream>

#include <ignition/physics/FindFeatures.hh>
#include <ignition/plugin/Loader.hh>
#include <ignition/physics/RequestEngine.hh>

#include <ignition/math/eigen3/Conversions.hh>

// Features
#include <ignition/physics/FrameSemantics.hh>
#include <ignition/physics/ForwardStep.hh>
#include <ignition/physics/GetEntities.hh>
#include <ignition/physics/Joint.hh>
#include <ignition/physics/sdf/ConstructModel.hh>
#include <ignition/physics/sdf/ConstructWorld.hh>

#include <limits>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include "test/Utils.hh"

using namespace ignition;

using TestFeatureList = ignition::physics::FeatureList<
  physics::ForwardStep,
  physics::GetEntities,
  physics::JointFrameSemantics,
  physics::LinkFrameSemantics,
  physics::sdf::ConstructSdfModel,
  physics::sdf::ConstructSdfWorld
>;

using TestEnginePtr = physics::Engine3dPtr<TestFeatureList>;

class KinematicsFeaturesFixture : public ::testing::Test
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

// Test joint frame semantics
TEST_F(KinematicsFeaturesFixture, JointFrameSemantics)
{
  sdf::Root root;
  const sdf::Errors errors = root.Load(TEST_WORLD_DIR "string_pendulum.sdf");
  ASSERT_TRUE(errors.empty()) << errors.front();

  auto world = this->engine->ConstructWorld(*root.WorldByIndex(0));
  ASSERT_NE(nullptr, world);

  auto model = world->GetModel("pendulum");
  ASSERT_NE(nullptr, model);
  auto pivotJoint = model->GetJoint("pivot");
  ASSERT_NE(nullptr, pivotJoint);
  auto childLink = model->GetLink("bob");
  ASSERT_NE(nullptr, childLink);

  physics::ForwardStep::Output output;
  physics::ForwardStep::State state;
  physics::ForwardStep::Input input;

  for (std::size_t i = 0; i < 100; ++i)
  {
    world->Step(output, state, input);
  }
  // Pose of Child link (C) in Joint frame (J)
  physics::Pose3d X_JC = physics::Pose3d::Identity();
  X_JC.translate(physics::Vector3d(0, 0, -1));

  // Notation: Using F_WJ for the frame data of frame J (joint) relative to
  // frame W (world).
  auto F_WJ = pivotJoint->FrameDataRelativeToWorld();
  physics::FrameData3d F_WCexpected = F_WJ;

  physics::Vector3d pendulumArmInWorld =
      F_WJ.pose.rotation() * X_JC.translation();

  F_WCexpected.pose = F_WJ.pose * X_JC;
  // angular acceleration of the child link is the same as the joint, so we 
  // don't need to assign a new value.

  // Note that the joint's linear velocity and linear acceleration are zero, so
  // they are omitted here.
  F_WCexpected.linearAcceleration =
      F_WJ.angularAcceleration.cross(pendulumArmInWorld) +
      F_WJ.angularVelocity.cross(
          F_WJ.angularVelocity.cross(pendulumArmInWorld));

  F_WCexpected.linearVelocity = F_WJ.angularVelocity.cross(pendulumArmInWorld);

  auto childLinkFrameData = childLink->FrameDataRelativeToWorld();
  EXPECT_TRUE(
      physics::test::Equal(F_WCexpected, childLinkFrameData, 1e-6));
}

/////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

