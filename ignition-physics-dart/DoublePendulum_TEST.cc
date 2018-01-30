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

#include <ignition/math/PID.hh>

#include <ignition/common/PluginLoader.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/SpecializedPluginPtr.hh>

#include <ignition/physics/ForwardStep.hh>

#include <utils/test_config.h>


using PhysicsPlugin = ignition::common::SpecializedPluginPtr<
    ignition::physics::ForwardStep,
    ignition::physics::SetState>;

/////////////////////////////////////////////////
TEST(DoublePendulum, Step)
{
  std::string projectPath = PROJECT_BINARY_PATH;

  ignition::common::SystemPaths sp;
  sp.AddPluginPaths(projectPath + "/ignition-physics-dart");
  std::string path = sp.FindSharedLibrary("ignition-physics-dart");

  ignition::common::PluginLoader loader;
  loader.LoadLibrary(path);

  PhysicsPlugin plugin = loader.Instantiate(
        "ignition::physics::dart::DoublePendulum");

  ASSERT_TRUE(plugin);

  ignition::physics::ForwardStep *step =
      plugin->QueryInterface<ignition::physics::ForwardStep>();

  ignition::physics::ForwardStep::State state;
  ignition::physics::ForwardStep::Output output;
  ignition::physics::ForwardStep::Input input;

  const std::chrono::duration<double> dt(std::chrono::milliseconds(1));
  ignition::physics::TimeStep &timeStep =
      input.Get<ignition::physics::TimeStep>();
  timeStep.dt = dt.count();

  ignition::physics::GeneralizedParameters &efforts =
      input.Get<ignition::physics::GeneralizedParameters>();
  efforts.dofs.push_back(0);
  efforts.dofs.push_back(1);
  efforts.forces.push_back(0.0);
  efforts.forces.push_back(0.0);

  // No input on the first step, let's just see the output
  step->Step(output, state, input);

  EXPECT_TRUE(output.Has<ignition::physics::JointPositions>());
  auto positions0 = output.Get<ignition::physics::JointPositions>();

  // the double pendulum is initially fully inverted
  // and angles are defined as zero in this state
  double angle00 = positions0.positions[positions0.dofs[0]];
  double angle01 = positions0.positions[positions0.dofs[1]];
  EXPECT_NEAR(0.0, angle00, 1e-6);
  EXPECT_NEAR(0.0, angle01, 1e-6);

  // set target with joint1 still inverted, but joint2 pointed down
  // this is also an equilibrium position
  const double target10 = 0.0;
  const double target11 = IGN_PI;

  // PID gains tuned in gazebo
  ignition::math::PID pid0(100, 0, 10);
  ignition::math::PID pid1(10, 0, 5);
  const std::chrono::duration<double> settleTime(std::chrono::seconds(4));
  unsigned int settleSteps = settleTime / dt;
  for (unsigned int i = 0; i < settleSteps; ++i)
  {
    auto positions = output.Get<ignition::physics::JointPositions>();
    double error0 = positions.positions[positions.dofs[0]] - target10;
    double error1 = positions.positions[positions.dofs[1]] - target11;

    efforts.forces[0] = pid0.Update(error0, dt);
    efforts.forces[1] = pid1.Update(error1, dt);

    step->Step(output, state, input);
  }

  // expect joints are near target positions
  EXPECT_TRUE(output.Has<ignition::physics::JointPositions>());
  auto positions1 = output.Get<ignition::physics::JointPositions>();
  double angle10 = positions1.positions[positions1.dofs[0]];
  double angle11 = positions1.positions[positions1.dofs[1]];
  EXPECT_NEAR(target10, angle10, 1e-5);
  EXPECT_NEAR(target11, angle11, 1e-3);

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
  ignition::physics::SetState *setState =
      plugin->QueryInterface<ignition::physics::SetState>();
  ASSERT_TRUE(setState);
  setState->SetStateTo(bookmark);

  step->Step(output, state, input);
  err = target.pose.Pos().Distance(ee_pose.pose.Pos());

  // The new error output should match the history perfectly, even though they
  // use floating point values
  EXPECT_DOUBLE_EQ(errorHistory[0], err);

  for (std::size_t i=0; i < Iterations; ++i)
  {
    step->Step(output, state, input);
    err = target.pose.Pos().Distance(ee_pose.pose.Pos());

    EXPECT_DOUBLE_EQ(errorHistory[i+1], err);
  }
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
