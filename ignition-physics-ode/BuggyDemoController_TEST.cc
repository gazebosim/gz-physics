/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <ignition/common/PluginLoader.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/SpecializedPluginPtr.hh>

#include <ignition/physics/ForwardStep.hh>

#include <utils/test_config.h>


using PhysicsPlugin = ignition::common::SpecializedPluginPtr<
  ignition::physics::ForwardStep,
  ignition::physics::SetState>;

/////////////////////////////////////////////////
TEST(BuggyDemoController, Step)
{
  std::string projectPath = PROJECT_BINARY_PATH;

  ignition::common::SystemPaths sp;
  sp.AddPluginPaths(projectPath + "/ignition-physics-ode");
  std::string path = sp.FindSharedLibrary("ignition-physics-ode");

  ignition::common::PluginLoader loader;
  loader.LoadLibrary(path);

  PhysicsPlugin plugin = loader.Instantiate(
        "ignition::physics::ode::BuggyDemoController");
  ASSERT_TRUE(plugin);

  ignition::physics::ForwardStep *step =
      plugin->QueryInterface<ignition::physics::ForwardStep>();

  ignition::physics::ForwardStep::State state;
  ignition::physics::ForwardStep::Output output;
  ignition::physics::ForwardStep::Input input;

  // No input on the first step
  step->Step(output, state, input);

  EXPECT_TRUE(output.Has<ignition::physics::WorldPoses>());
  EXPECT_TRUE(output.Has<ignition::physics::ChassisPose>());
  EXPECT_TRUE(output.Has<ignition::physics::FrontWheelPose>());


  ignition::physics::ApplySpeedSteer &speedSteer =
    input.Get<ignition::physics::ApplySpeedSteer>();
  speedSteer.speed = 0.5;
  speedSteer.steer = 0.0;
  ignition::physics::ChassisPose &chPose =
    output.Get<ignition::physics::ChassisPose>();
  ignition::physics::FrontWheelPose &wheelPose =
    output.Get<ignition::physics::FrontWheelPose>();

  // Take 5000 steps for the buggy to run without steering
  for (std::size_t i=0; i<5000; ++i)
  {
    step->Step(output, state, input);
  }

  ignition::physics::ForwardStep::State bookmark = state;
  ignition::math::Pose3d bookmarkChassisPose = chPose.pose;
  ignition::math::Pose3d bookmarkWheelPose = wheelPose.pose;

  // Expect chassis position Y() is near 0, since steer = 0
  EXPECT_NEAR(chPose.pose.Pos().Y(), .0, 1e-3);
  // Expect chassis position Z() is near 0.25, half the height of chassis
  EXPECT_NEAR(chPose.pose.Pos().Z(), .25, .01);
  // Expect difference of wheel and chassis position X() is near
  // half length of chasis
  EXPECT_NEAR(wheelPose.pose.Pos().X(), chPose.pose.Pos().X(), .35);
  // Expect front wheel position Y() is near 0, since steer = 0
  EXPECT_NEAR(wheelPose.pose.Pos().Y(), .0, 1e-3);
  // Expect difference of front wheel and chassis position Z() is near
  // (0.5*height-wheelradius = 0.07), the error for this one is larger
  // due to the contact with ground.
  EXPECT_NEAR(wheelPose.pose.Pos().Z(), chPose.pose.Pos().Z()-0.07, .1);

  // Update steer to be nonzero
  speedSteer.speed = 1.0;
  speedSteer.steer = 0.1;
  for (std::size_t i = 0; i < 2000; ++i)
  {
    step->Step(output, state, input);
  }
  EXPECT_GT(chPose.pose.Pos().X(), bookmarkChassisPose.Pos().X());
  EXPECT_GT(wheelPose.pose.Pos().X(), bookmarkWheelPose.Pos().X());

  // Go back to the bookmarked state and run another 5000 steps
  ignition::physics::SetState *setState =
    plugin->QueryInterface<ignition::physics::SetState>();
  ASSERT_TRUE(setState);
  setState->SetStateTo(bookmark);

  step->Step(output, state, input);
  EXPECT_LT(bookmarkChassisPose.Pos().Distance(chPose.pose.Pos()), 1e-2);
  EXPECT_LT(bookmarkWheelPose.Pos().Distance(wheelPose.pose.Pos()), 1e-2);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
