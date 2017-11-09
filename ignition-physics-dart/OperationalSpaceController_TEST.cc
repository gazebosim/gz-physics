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
    ignition::physics::ForwardStep::Engine<void>,
    ignition::physics::SetState::Engine<void>>;

/////////////////////////////////////////////////
TEST(OperationalSpaceController, Step)
{
  std::string projectPath = PROJECT_BINARY_PATH;

  ignition::common::SystemPaths sp;
  sp.AddPluginPaths(projectPath + "/ignition-physics-dart");
  std::string path = sp.FindSharedLibrary("ignition-physics-dart");

  ignition::common::PluginLoader loader;
  loader.LoadLibrary(path);

  PhysicsPlugin plugin = loader.Instantiate(
        "ignition::physics::dart::OperationalSpaceController");

  ASSERT_TRUE(plugin);

  ignition::physics::ForwardStep::Engine<void> *step =
      plugin->GetInterface<ignition::physics::ForwardStep::Engine<void>>();

  ignition::physics::ForwardStep::State state;
  ignition::physics::ForwardStep::Output output;
  ignition::physics::ForwardStep::Input input;

  // No input on the first step, let's just see the output
  step->Step(output, state, input);

  EXPECT_TRUE(output.Has<ignition::physics::WorldPoses>());
  EXPECT_TRUE(output.Has<ignition::physics::EndEffectorPose>());

  // Now that we have the first output, we'll set the input's target to the pose
  // that the output object reported for the end effector
  ignition::physics::TargetPose &target =
      input.Get<ignition::physics::TargetPose>();

  static_cast<ignition::physics::WorldPose&>(target) =
      output.Get<ignition::physics::EndEffectorPose>();

  // We'll take another step, keeping the target the same
  step->Step(output, state, input);

  ignition::physics::EndEffectorPose &ee_pose =
      output.Get<ignition::physics::EndEffectorPose>();

  // The error between the target and the end effector should be pretty low
  double err = target.pose.Pos().Distance(ee_pose.pose.Pos());
  EXPECT_LT(err, 1e-3);

  // Move the target down 5cm
  const double dz = 0.05;
  target.pose.Pos().Z() -= dz;

  double last_err = err;
  step->Step(output, state, input);
  err = target.pose.Pos().Distance(ee_pose.pose.Pos());

  // We expect a larger error because we have spontaneously moved the target
  // location, and the end effector will need time to adjust
  EXPECT_GT(err, last_err);
  EXPECT_NEAR(dz, err, 1e-3);

  const std::size_t Iterations = 1000;

  last_err = err;
  for (std::size_t i=0; i < Iterations; ++i)
  {
    step->Step(output, state, input);
    err = target.pose.Pos().Distance(ee_pose.pose.Pos());

    // Now we expect the error to converge towards zero
    EXPECT_LE(err, last_err);
    last_err = err;
  }

  EXPECT_LT(err, 1e-3);

  ignition::physics::ForwardStep::State bookmark = state;
  std::vector<double> errorHistory;

  const double dy = 0.05;
  target.pose.Pos().Y() += dy;

  step->Step(output, state, input);
  err = target.pose.Pos().Distance(ee_pose.pose.Pos());
  EXPECT_GT(err, last_err);

  errorHistory.push_back(err);

  last_err = err;
  for (std::size_t i=0; i < Iterations; ++i)
  {
    step->Step(output, state, input);
    err = target.pose.Pos().Distance(ee_pose.pose.Pos());

    EXPECT_LE(err, last_err);
    last_err = err;

    errorHistory.push_back(err);
  }

  EXPECT_LT(err, 1e-3);

  // Go back to the bookmarked state and run through the steps again.
  ignition::physics::SetState::Engine<void> *setState =
      plugin->GetInterface<ignition::physics::SetState::Engine<void>>();
  ASSERT_TRUE(setState);
  setState->SetStateTo(bookmark);

  step->Step(output, state, input);
  err = target.pose.Pos().Distance(ee_pose.pose.Pos());

  // The new error output should match the history perfectly, even though they
  // use floating point values
  EXPECT_EQ(errorHistory[0], err);

  for (std::size_t i=0; i < Iterations; ++i)
  {
    step->Step(output, state, input);
    err = target.pose.Pos().Distance(ee_pose.pose.Pos());

    EXPECT_EQ(errorHistory[i+1], err);
  }
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
