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
/// \brief Helper function for comparing WorldPoses objects
/// \return true if objects are equal.
bool WorldPosesEqual(const ignition::physics::WorldPoses &_poses1,
                     const ignition::physics::WorldPoses &_poses2)
{
  bool result = true;
  result = result && (_poses1.annotation ==
                      _poses2.annotation);
  result = result && (_poses1.entries.size() ==
                      _poses2.entries.size());
  if (result)
  {
    for (std::size_t i = 0; i < _poses1.entries.size(); ++i)
    {
      result = result && (_poses1.entries[i].body ==
                          _poses2.entries[i].body);
      result = result && (_poses1.entries[i].pose ==
                          _poses2.entries[i].pose);
    }
  }
  return result;
}

/////////////////////////////////////////////////
/// \brief Helper function for comparing WorldPoses objects
/// \return true if objects are equal except for the
/// Z position having decremented by 1.0.
bool WorldPosesFell(const ignition::physics::WorldPoses &_poses1,
                     const ignition::physics::WorldPoses &_poses2)
{
  bool result = true;
  result = result && (_poses1.annotation ==
                      _poses2.annotation);
  result = result && (_poses1.entries.size() ==
                      _poses2.entries.size());
  if (result)
  {
    for (std::size_t i = 0; i < _poses1.entries.size(); ++i)
    {
      result = result && (_poses1.entries[i].body ==
                          _poses2.entries[i].body);
      result = result && (_poses1.entries[i].pose.Rot() ==
                          _poses2.entries[i].pose.Rot());
      result = result && ignition::math::equal(
          _poses1.entries[i].pose.Pos().X(),
          _poses2.entries[i].pose.Pos().X());
      result = result && ignition::math::equal(
          _poses1.entries[i].pose.Pos().Y(),
          _poses2.entries[i].pose.Pos().Y());
      result = result && ignition::math::equal(
          _poses1.entries[i].pose.Pos().Z(),
          _poses2.entries[i].pose.Pos().Z() + 1.0);
    }
  }
  return result;
}

/////////////////////////////////////////////////
TEST(MinimalFallingScene, Step)
{
  std::string projectPath = PROJECT_BINARY_PATH;

  ignition::common::SystemPaths sp;
  sp.AddPluginPaths(projectPath + "/ignition-physics-none");
  std::string path = sp.FindSharedLibrary("ignition-physics-none");

  ignition::common::PluginLoader loader;
  loader.LoadLibrary(path);

  PhysicsPlugin plugin = loader.Instantiate(
        "ignition::physics::none::MinimalFallingScene");

  ASSERT_TRUE(plugin);

  ignition::physics::ForwardStep *step =
      plugin->QueryInterface<ignition::physics::ForwardStep>();

  ignition::physics::ForwardStep::State state;
  ignition::physics::ForwardStep::Output output;
  ignition::physics::ForwardStep::Input input;

  // No input on the first step, let's just see the output
  step->Step(output, state, input);

  EXPECT_TRUE(output.Has<ignition::physics::WorldPoses>());
  auto poses0 = output.Get<ignition::physics::WorldPoses>();

  // expect state to match the output poses
  EXPECT_TRUE(
    WorldPosesEqual(poses0, state.Get<ignition::physics::WorldPoses>()));

  auto lastPoses = poses0;
  // Take another few steps
  for (unsigned int i = 0; i < 10; ++i)
  {
    step->Step(output, state, input);

    // expect state and output to match poses0
    EXPECT_TRUE(
      WorldPosesFell(lastPoses, state.Get<ignition::physics::WorldPoses>()));
    EXPECT_TRUE(
      WorldPosesFell(lastPoses, output.Get<ignition::physics::WorldPoses>()));

    lastPoses = output.Get<ignition::physics::WorldPoses>();
  }

  // make a copy of poses and modify them
  ignition::physics::WorldPoses newPoses = poses0;
  newPoses.annotation = "modified";
  size_t poseCount = newPoses.entries.size();
  ignition::physics::WorldPose newPose;
  newPose.body = poseCount++;
  newPose.pose = ignition::math::Pose3d(1, 4, 9, 0.1, -0.2, 0.3);
  newPoses.entries.push_back(newPose);

  // confirm that poses are modified
  EXPECT_FALSE(
    WorldPosesEqual(poses0, newPoses));

  // set state to new poses
  {
    ignition::physics::SetState *setState =
        plugin->QueryInterface<ignition::physics::SetState>();
    ASSERT_TRUE(setState);
    ignition::physics::ForwardStep::State newState;
    auto &newStateWorldPoses = newState.Get<ignition::physics::WorldPoses>();
    newStateWorldPoses = newPoses;
    setState->SetStateTo(newState);
  }

  lastPoses = newPoses;
  // Take another few steps and expect that it matches newPoses
  for (unsigned int i = 0; i < 10; ++i)
  {
    step->Step(output, state, input);

    // expect state and output to match poses0
    EXPECT_TRUE(
      WorldPosesFell(lastPoses, state.Get<ignition::physics::WorldPoses>()));
    EXPECT_TRUE(
      WorldPosesFell(lastPoses, output.Get<ignition::physics::WorldPoses>()));

    lastPoses = output.Get<ignition::physics::WorldPoses>();
  }
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
